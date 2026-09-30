"""Train a bounded residual SAC actor and export it for the NumPy race driver.

Run only outside scored evaluations, with AutoDRIVE connected, the bridge up,
and no manual driver or referee. The simulator build is user-declared because
AutoDRIVE does not expose a trustworthy build identifier over ROS.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import stable_baselines3
import torch
import yaml
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback

from team_driver.residual_policy import (
    MODEL_FORMAT_VERSION, MODEL_FORMAT_VERSION_V2,
    OBSERVATION_DIM, OBSERVATION_DIM_V2,
    OBSERVATION_VERSION, OBSERVATION_VERSION_V2,
    STACKED_STEERING_SCALE, STACKED_SPEED_SCALE,
    ResidualPolicy, decode_action,
)
from training.gym_env import AutoDriveResidualEnv, REWARD_WEIGHTS


class ZeroStartSAC(SAC):
    """Gather baseline transitions before SAC's random exploration starts."""

    def _sample_action(self, learning_starts, action_noise=None, n_envs=1):
        if self.num_timesteps < learning_starts:
            zero = np.zeros((n_envs, 2), dtype=np.float32)
            return zero.copy(), zero.copy()
        return super()._sample_action(learning_starts, action_noise, n_envs)


def export_actor(model: SAC, path: Path) -> None:
    """Write a deterministic actor with the matching observation version."""
    actor = model.policy.actor
    observation_dim = int(model.observation_space.shape[0])
    if observation_dim not in (OBSERVATION_DIM, OBSERVATION_DIM_V2):
        raise ValueError("unsupported actor observation dimension")
    version = (OBSERVATION_VERSION_V2 if observation_dim == OBSERVATION_DIM_V2
               else OBSERVATION_VERSION)
    format_version = (MODEL_FORMAT_VERSION_V2 if version == 2
                      else MODEL_FORMAT_VERSION)
    layers = (actor.latent_pi[0], actor.latent_pi[2], actor.mu)
    arrays = {}
    for index, layer in enumerate(layers):
        arrays[f"w{index}"] = layer.weight.detach().cpu().numpy().astype(np.float32)
        arrays[f"b{index}"] = layer.bias.detach().cpu().numpy().astype(np.float32)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp.npz")
    np.savez_compressed(
        temporary, format_version=np.int64(format_version),
        observation_version=np.int64(version), **arrays,
    )
    runtime = ResidualPolicy.load(temporary)
    # The runtime applies decode_action after tanh. Compare physical controls,
    # including on nonzero states, before accepting an export.
    rng = np.random.default_rng(9182)
    for observation in [np.zeros(observation_dim, dtype=np.float32),
                        *rng.uniform(-1, 1, (8, observation_dim)).astype(np.float32)]:
        raw, _ = model.predict(observation, deterministic=True)
        expected = np.asarray(decode_action(raw))
        actual = np.asarray(runtime.predict(observation))
        if not np.allclose(actual, expected, atol=2e-6, rtol=2e-6):
            temporary.unlink(missing_ok=True)
            raise RuntimeError("exported actor does not match SAC inference")
    os.replace(temporary, path)


class RunRecorder(BaseCallback):
    def __init__(self, run_dir: Path, settings: dict, checkpoint_every: int):
        super().__init__()
        self.run_dir = run_dir
        self.settings = settings
        self.checkpoint_every = checkpoint_every
        self.started = time.monotonic()
        self.episode_reward = 0.0
        self.episode_steps = 0
        self.episode_progress_m = 0.0
        self.episodes = 0
        self.last_saved_step = -1

    def _on_step(self) -> bool:
        reward = float(self.locals["rewards"][0])
        info = self.locals["infos"][0]
        self.episode_reward += reward
        self.episode_steps += 1
        self.episode_progress_m += float(info.get("progress_m", 0.0))
        if bool(self.locals["dones"][0]):
            record = {
                "episode": self.episodes, "step": self.num_timesteps,
                "reward": self.episode_reward, "steps": self.episode_steps,
                "progress_m": self.episode_progress_m,
                "reason": info.get("reason"),
                "collisions": info.get("collisions", 0),
                "laps": info.get("laps", 0),
                "elapsed_s": round(time.monotonic() - self.started, 3),
            }
            with (self.run_dir / "episodes.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, sort_keys=True) + "\n")
            print(json.dumps(record, sort_keys=True), flush=True)
            self.episodes += 1
            self.episode_reward = 0.0
            self.episode_steps = 0
            self.episode_progress_m = 0.0
        if self.num_timesteps % self.checkpoint_every == 0:
            self.save_checkpoint()
        return True

    def save_checkpoint(self, *, label: str | None = None) -> None:
        step = self.model.num_timesteps
        if label is None and step == self.last_saved_step:
            return
        name = label or f"step_{step:07d}"
        checkpoint = self.run_dir / name
        self.model.save(str(checkpoint.with_suffix(".zip")))
        if label in ("final", "interrupted"):
            self.model.save_replay_buffer(str(checkpoint.with_suffix(".replay.pkl")))
        export_actor(self.model, checkpoint.with_suffix(".npz"))
        metadata = {
            **self.settings,
            "checkpoint": name,
            "steps": step,
            "episodes_completed": self.episodes,
            "duration_s": round(time.monotonic() - self.started, 3),
            "saved_at_utc": datetime.now(timezone.utc).isoformat(),
            "reward_weights": REWARD_WEIGHTS,
            "torch_version": torch.__version__,
            "stable_baselines3_version": stable_baselines3.__version__,
            "runtime_format_version": (MODEL_FORMAT_VERSION_V2
                                       if self.settings["observation_version"] == 2
                                       else MODEL_FORMAT_VERSION),
            "observation_version": self.settings["observation_version"],
        }
        checkpoint.with_suffix(".json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.last_saved_step = step
        print(f"Saved {checkpoint.name} at {step} steps", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-mode", action="store_true", required=True,
                        help="explicitly authorize training resets outside a scored run")
    parser.add_argument("--simulator-variant", required=True,
                        choices=("practice", "compete"))
    parser.add_argument("--simulator-build", required=True,
                        help="name/version of the executable actually open in AutoDRIVE")
    parser.add_argument("--centerline", type=Path)
    parser.add_argument("--steps", type=int, default=20000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--baseline-steps", type=int, default=500)
    parser.add_argument("--checkpoint-every", type=int, default=2000)
    parser.add_argument("--frame-skip", type=int, default=4)
    parser.add_argument("--max-episode-steps", type=int,
                        help="defaults to 2,000 simulator frames per episode")
    parser.add_argument("--lookahead-min", type=float, default=0.55)
    parser.add_argument("--lookahead-max", type=float, default=1.8)
    parser.add_argument("--lookahead-speed-gain", type=float, default=0.35)
    parser.add_argument("--target-speed", type=float, default=4.4)
    parser.add_argument("--max-lateral-accel", type=float, default=1.5)
    parser.add_argument("--max-braking-decel", type=float, default=2.0)
    parser.add_argument("--safety-max-deceleration", type=float, default=2.0)
    parser.add_argument("--safety-horizon", type=float, default=5.5)
    parser.add_argument("--vehicle-limits-path", type=Path)
    parser.add_argument("--map-guard-path",
                        help="packaged, or the map YAML path used by the race driver")
    parser.add_argument("--planning-lateral-mps2", type=float, default=0.0)
    parser.add_argument("--planning-top-speed-mps", type=float, default=0.0)
    parser.add_argument("--observation-version", type=int, choices=(1, 2), default=1)
    parser.add_argument("--output-dir", type=Path,
                        default=Path(__file__).resolve().parent / "runs")
    parser.add_argument("--run-id", help="directory name; defaults to UTC time and seed")
    parser.add_argument("--resume", type=Path,
                        help="SAC checkpoint to continue from with a fresh replay buffer")
    parser.add_argument("--baseline-config", type=Path,
                        help="load the classical driver's matching training settings")
    parser.add_argument("--base-policy-path", type=Path,
                        help="fixed actor whose output is the new residual's baseline")
    parser.add_argument("--continuous-control", action="store_true",
                        help="update the fixed actor on every LiDAR frame while SAC learns")
    parser.add_argument("--stacked-steering-scale", type=float,
                        default=STACKED_STEERING_SCALE)
    parser.add_argument("--stacked-speed-scale", type=float,
                        default=STACKED_SPEED_SCALE)
    args = parser.parse_args()
    if args.baseline_config is not None:
        payload = yaml.safe_load(args.baseline_config.read_text(encoding="utf-8"))
        params = payload["driver"]["ros__parameters"]
        if (not params.get("baseline_only") or not params.get("safety_enabled")
                or params.get("wheelbase") != 0.324
                or params.get("safety_side_margin") != 0.08
                or params.get("safety_stop_margin") != 0.25
                or params.get("speed_ff_gain") != 0.16
                or params.get("speed_kp") != 0.24
                or params.get("speed_ki") != 0.08
                or params.get("max_range") != 10.0):
            parser.error("baseline config uses controller settings unsupported by the training environment")
        args.centerline = Path(params["centerline_path"]) if params["centerline_path"] else None
        args.vehicle_limits_path = (
            args.baseline_config.parent / "vehicle_limits.yaml"
            if params["vehicle_limits_path"] == "packaged"
            else Path(params["vehicle_limits_path"])
        )
        args.map_guard_path = params["map_guard_path"]
        for key in ("planning_lateral_mps2", "planning_top_speed_mps",
                    "lookahead_min", "lookahead_max", "lookahead_speed_gain",
                    "safety_max_deceleration", "safety_horizon"):
            setattr(args, key, float(params[key]))
    if args.steps < 1 or args.baseline_steps < 1 or args.checkpoint_every < 1:
        parser.error("steps, baseline-steps and checkpoint-every must be positive")
    if args.baseline_steps >= args.steps:
        parser.error("baseline-steps must be less than total steps")
    if args.frame_skip < 1:
        parser.error("frame-skip must be positive")
    if args.max_episode_steps is None:
        args.max_episode_steps = math.ceil(2000 / args.frame_skip)
    if args.max_episode_steps < 1:
        parser.error("frame-skip and max-episode-steps must be positive")
    if not all(np.isfinite(value) and value > 0.0 for value in (
            args.target_speed, args.max_lateral_accel, args.max_braking_decel,
            args.safety_horizon, args.safety_max_deceleration,
            args.lookahead_min, args.lookahead_max)):
        parser.error("speed and acceleration limits must be finite and positive")
    if args.lookahead_max < args.lookahead_min or not np.isfinite(args.lookahead_speed_gain) or args.lookahead_speed_gain < 0:
        parser.error("lookahead bounds or gain are invalid")
    if (not np.isfinite(args.planning_lateral_mps2) or args.planning_lateral_mps2 < 0
            or not np.isfinite(args.planning_top_speed_mps)
            or args.planning_top_speed_mps < 0):
        parser.error("planning overrides must be finite and nonnegative")
    if args.observation_version == 2 and args.vehicle_limits_path is None:
        parser.error("v2 training requires --vehicle-limits-path")
    if args.simulator_variant == "practice" and args.centerline is None:
        parser.error("practice training needs --centerline for the practice map")
    if args.resume is not None and not args.resume.is_file():
        parser.error(f"resume checkpoint does not exist: {args.resume}")
    if args.base_policy_path is not None and not args.base_policy_path.is_file():
        parser.error(f"base policy does not exist: {args.base_policy_path}")
    if not all(np.isfinite(value) and 0.0 <= value <= 1.0 for value in
               (args.stacked_steering_scale, args.stacked_speed_scale)):
        parser.error("stacked action scales must be within [0, 1]")
    return args


def main() -> None:
    args = parse_args()
    run_id = args.run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + f"_s{args.seed}"
    if Path(run_id).name != run_id or run_id in (".", ".."):
        raise ValueError("run-id must be a single directory name")
    run_dir = args.output_dir.resolve() / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    settings = {
        "seed": args.seed, "simulator_variant": args.simulator_variant,
        "simulator_build": args.simulator_build,
        "centerline": str(args.centerline.resolve()) if args.centerline else "packaged compete racing line",
        "requested_steps": args.steps, "baseline_steps": args.baseline_steps,
        "checkpoint_every": args.checkpoint_every, "frame_skip": args.frame_skip,
        "max_episode_steps": args.max_episode_steps,
        "lookahead_min": args.lookahead_min,
        "lookahead_max": args.lookahead_max,
        "lookahead_speed_gain": args.lookahead_speed_gain,
        "target_speed": args.target_speed,
        "max_lateral_accel": args.max_lateral_accel,
        "max_braking_decel": args.max_braking_decel,
        "safety_horizon": args.safety_horizon,
        "safety_max_deceleration": args.safety_max_deceleration,
        "baseline_config": str(args.baseline_config.resolve()) if args.baseline_config else None,
        "baseline_config_sha256": (hashlib.sha256(args.baseline_config.read_bytes()).hexdigest()
                                   if args.baseline_config else None),
        "vehicle_limits_path": str(args.vehicle_limits_path.resolve()) if args.vehicle_limits_path else None,
        "map_guard_path": args.map_guard_path,
        "planning_lateral_mps2": args.planning_lateral_mps2,
        "planning_top_speed_mps": args.planning_top_speed_mps,
        "observation_version": args.observation_version,
        "resumed_from": str(args.resume.resolve()) if args.resume else None,
        "base_policy_path": str(args.base_policy_path.resolve()) if args.base_policy_path else None,
        "continuous_control": args.continuous_control,
        "base_policy_sha256": (hashlib.sha256(args.base_policy_path.read_bytes()).hexdigest()
                               if args.base_policy_path else None),
        "stacked_steering_scale": (args.stacked_steering_scale if args.base_policy_path else None),
        "stacked_speed_scale": (args.stacked_speed_scale if args.base_policy_path else None),
        "replay_buffer_restored": False,
        "algorithm": "SAC", "actor_layers": [
            OBSERVATION_DIM_V2 if args.observation_version == 2 else OBSERVATION_DIM,
            64, 64, 2],
        "run_started_utc": datetime.now(timezone.utc).isoformat(),
    }
    env = None
    recorder = None
    try:
        env = AutoDriveResidualEnv(
            simulator_variant=args.simulator_variant, training_mode=args.training_mode,
            centerline_path=args.centerline, frame_skip=args.frame_skip,
            max_steps=args.max_episode_steps, target_speed=args.target_speed,
            max_lateral_accel=args.max_lateral_accel,
            max_braking_decel=args.max_braking_decel,
            safety_horizon=args.safety_horizon,
            safety_max_deceleration=args.safety_max_deceleration,
            lookahead_min=args.lookahead_min,
            lookahead_max=args.lookahead_max,
            lookahead_speed_gain=args.lookahead_speed_gain,
            vehicle_limits_path=args.vehicle_limits_path,
            map_guard_path=args.map_guard_path,
            planning_lateral_mps2=args.planning_lateral_mps2,
            planning_top_speed_mps=args.planning_top_speed_mps,
            observation_version=args.observation_version,
            base_policy_path=args.base_policy_path,
            continuous_control=args.continuous_control,
            stacked_steering_scale=args.stacked_steering_scale,
            stacked_speed_scale=args.stacked_speed_scale,
        )
        settings["effective_target_speed_mps"] = env.target_speed
        settings["effective_lateral_mps2"] = env.max_lateral_accel
        if args.resume is None:
            model = ZeroStartSAC(
                "MlpPolicy", env, policy_kwargs={"net_arch": {"pi": [64, 64], "qf": [64, 64]}},
                learning_starts=args.baseline_steps, buffer_size=50000,
                batch_size=64, train_freq=1, gradient_steps=1,
                seed=args.seed, device="cpu", verbose=0,
            )
            with torch.no_grad():
                model.policy.actor.mu.weight.zero_()
                model.policy.actor.mu.bias.zero_()
        else:
            model = ZeroStartSAC.load(args.resume, env=env, device="cpu")
            if model.observation_space.shape != env.observation_space.shape:
                raise ValueError("resume checkpoint observation shape differs from environment")
            model.learning_starts = args.baseline_steps
            model.set_random_seed(args.seed)
            replay_path = args.resume.with_suffix(".replay.pkl")
            if replay_path.is_file():
                model.load_replay_buffer(str(replay_path))
                settings["replay_buffer_restored"] = True
            settings["checkpoint_start_timesteps"] = model.num_timesteps
            print("Continuing checkpoint weights; replay buffer restored: "
                  f"{settings['replay_buffer_restored']}", flush=True)
        recorder = RunRecorder(run_dir, settings, args.checkpoint_every)
        recorder.model = model
        recorder.save_checkpoint(label="resume_start" if args.resume else "baseline")
        model.learn(total_timesteps=args.steps, callback=recorder,
                    reset_num_timesteps=True, progress_bar=False)
        recorder.save_checkpoint(label="final")
    except KeyboardInterrupt:
        if recorder is not None:
            recorder.save_checkpoint(label="interrupted")
        print("Training interrupted; latest checkpoint saved.", flush=True)
    finally:
        if env is not None:
            env.close()


if __name__ == "__main__":
    main()
