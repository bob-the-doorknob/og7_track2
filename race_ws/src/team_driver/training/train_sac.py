"""Train a bounded residual SAC actor and export it for the NumPy race driver.

Run only outside scored evaluations, with AutoDRIVE connected, the bridge up,
and no manual driver or referee. The simulator build is user-declared because
AutoDRIVE does not expose a trustworthy build identifier over ROS.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import stable_baselines3
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback

from team_driver.residual_policy import (
    MODEL_FORMAT_VERSION, OBSERVATION_VERSION, ResidualPolicy, decode_action,
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
    """Write the deterministic tanh(mu) actor in runtime's format v1."""
    actor = model.policy.actor
    layers = (actor.latent_pi[0], actor.latent_pi[2], actor.mu)
    arrays = {}
    for index, layer in enumerate(layers):
        arrays[f"w{index}"] = layer.weight.detach().cpu().numpy().astype(np.float32)
        arrays[f"b{index}"] = layer.bias.detach().cpu().numpy().astype(np.float32)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp.npz")
    np.savez_compressed(
        temporary, format_version=np.int64(MODEL_FORMAT_VERSION),
        observation_version=np.int64(OBSERVATION_VERSION), **arrays,
    )
    runtime = ResidualPolicy.load(temporary)
    # The runtime applies decode_action after tanh. Compare physical controls,
    # including on nonzero states, before accepting an export.
    rng = np.random.default_rng(9182)
    for observation in [np.zeros(24, dtype=np.float32),
                        *rng.uniform(-1, 1, (8, 24)).astype(np.float32)]:
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
            "runtime_format_version": MODEL_FORMAT_VERSION,
            "observation_version": OBSERVATION_VERSION,
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
    parser.add_argument("--max-episode-steps", type=int, default=500)
    parser.add_argument("--target-speed", type=float, default=3.0)
    parser.add_argument("--max-lateral-accel", type=float, default=1.5)
    parser.add_argument("--max-braking-decel", type=float, default=2.0)
    parser.add_argument("--output-dir", type=Path, default=Path("training/runs"))
    parser.add_argument("--run-id", help="directory name; defaults to UTC time and seed")
    args = parser.parse_args()
    if args.steps < 1 or args.baseline_steps < 1 or args.checkpoint_every < 1:
        parser.error("steps, baseline-steps and checkpoint-every must be positive")
    if args.baseline_steps >= args.steps:
        parser.error("baseline-steps must be less than total steps")
    if args.frame_skip < 1 or args.max_episode_steps < 1:
        parser.error("frame-skip and max-episode-steps must be positive")
    if not all(np.isfinite(value) and value > 0.0 for value in (
            args.target_speed, args.max_lateral_accel, args.max_braking_decel)):
        parser.error("speed and acceleration limits must be finite and positive")
    if args.simulator_variant == "practice" and args.centerline is None:
        parser.error("practice training needs --centerline for the practice map")
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
        "centerline": str(args.centerline.resolve()) if args.centerline else "packaged compete",
        "requested_steps": args.steps, "baseline_steps": args.baseline_steps,
        "checkpoint_every": args.checkpoint_every, "frame_skip": args.frame_skip,
        "max_episode_steps": args.max_episode_steps,
        "target_speed": args.target_speed,
        "max_lateral_accel": args.max_lateral_accel,
        "max_braking_decel": args.max_braking_decel,
        "algorithm": "SAC", "actor_layers": [24, 64, 64, 2],
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
        )
        model = ZeroStartSAC(
            "MlpPolicy", env, policy_kwargs={"net_arch": {"pi": [64, 64], "qf": [64, 64]}},
            learning_starts=args.baseline_steps, buffer_size=50000,
            batch_size=64, train_freq=1, gradient_steps=1,
            seed=args.seed, device="cpu", verbose=0,
        )
        with torch.no_grad():
            model.policy.actor.mu.weight.zero_()
            model.policy.actor.mu.bias.zero_()
        recorder = RunRecorder(run_dir, settings, args.checkpoint_every)
        recorder.model = model
        recorder.save_checkpoint(label="baseline")
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
