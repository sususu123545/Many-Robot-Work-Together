"""Score a fine-tuned SmolVLA checkpoint on held-out Gazebo episodes.

Training loss alone proves nothing: a policy that predicts "stay exactly where
you are" gets a low loss on smooth demonstrations while being useless.  So every
number here is reported next to a zero-motion baseline computed from the same
frames, and only beating that baseline means the model learned something.

Runs offline.  No Gazebo, no ROS, no robot: it reads a LeRobot dataset and a
checkpoint directory produced by lerobot-train.

Usage:
    python3 eval_smolvla_offline.py \
        --checkpoint <run>/checkpoints/003600/pretrained_model \
        --dataset-root <lerobot_gazebo_t10_2_camera_corrected> \
        --repo-id local/armpi_t07_sim \
        --episodes 3,13 --samples 64 --out report.json
"""
import argparse
import json
import time
from pathlib import Path

import torch
from torch.utils.data import DataLoader

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']


def load_policy(checkpoint):
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
    from lerobot.processor.pipeline import DataProcessorPipeline

    policy = SmolVLAPolicy.from_pretrained(checkpoint)
    policy.eval()
    if policy.config.device:
        policy.to(policy.config.device)

    pre = DataProcessorPipeline.from_pretrained(
        checkpoint, config_filename='policy_preprocessor.json')
    post = DataProcessorPipeline.from_pretrained(
        checkpoint, config_filename='policy_postprocessor.json')
    return policy, pre, post


def build_dataset(root, repo_id, episodes, chunk_size, fps):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(
        repo_id, root=root, episodes=episodes, video_backend='pyav',
        delta_timestamps={'action': [i / fps for i in range(chunk_size)]})
    if float(dataset.fps) != float(fps):
        raise SystemExit('dataset fps %s does not match the expected %s'
                         % (dataset.fps, fps))
    return dataset


def to_device(batch, device):
    return {k: (v.to(device) if torch.is_tensor(v) else v) for k, v in batch.items()}


def score(policy, pre, post, batch, chunk_size, seed):
    """Return per-sample L1 errors for the policy and for a zero-motion guess."""
    device = next(policy.parameters()).device
    batch = to_device(batch, device)

    observations = pre(batch)
    ground_truth = observations['action']          # already normalized by pre()

    generator = torch.Generator(device='cpu').manual_seed(seed)
    noise = torch.randn((ground_truth.shape[0], chunk_size, policy.config.max_action_dim),
                        generator=generator).to(device)

    with torch.no_grad():
        predicted = policy.predict_action_chunk(observations, noise=noise)

    valid = batch.get('action_is_pad')
    if valid is not None:
        keep = ~valid.to(device)
    else:
        keep = torch.ones(ground_truth.shape[:2], dtype=torch.bool, device=device)

    # Normalized-space error, so the number is comparable across checkpoints
    # regardless of the raw unit of any single joint.
    diff = (predicted[:, :, :ground_truth.shape[-1]] - ground_truth).abs()
    masked = diff[keep]
    normalized_l1 = masked.mean().item()

    # Zero-motion baseline: repeat the first ground-truth action for all steps.
    hold = ground_truth[:, :1, :].expand_as(ground_truth)
    baseline_l1 = (hold - ground_truth).abs()[keep].mean().item()

    # Raw units, per joint, so the report is readable by a human looking at the
    # arm.  The postprocessor is a pipeline and wants a transition dict, not a
    # bare tensor, and it moves its result to CPU.
    channels = ground_truth.shape[-1]
    with torch.no_grad():
        predicted_raw = post({'action': predicted[:, :, :channels]})['action'].cpu()
        truth_raw = post({'action': ground_truth})['action'].cpu()
    keep_cpu = keep.cpu()
    hold_raw = truth_raw[:, :1, :].expand_as(truth_raw)

    per_joint = []
    for index, name in enumerate(JOINT_NAMES[:truth_raw.shape[-1]]):
        error = (predicted_raw[:, :, index] - truth_raw[:, :, index]).abs()[keep_cpu]
        drift = (hold_raw[:, :, index] - truth_raw[:, :, index]).abs()[keep_cpu]
        per_joint.append({
            'joint': name,
            'policy_l1_raw': round(error.mean().item(), 5),
            'hold_l1_raw': round(drift.mean().item(), 5),
        })

    return {
        'normalized_l1': normalized_l1,
        'hold_normalized_l1': baseline_l1,
        'ratio_to_hold': (normalized_l1 / baseline_l1) if baseline_l1 > 0 else None,
        'per_joint_raw': per_joint,
        'masked_out_steps': int((~keep).sum().item()),
        'total_steps': int(keep.numel()),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--dataset-root', type=Path, required=True)
    parser.add_argument('--repo-id', default='local/armpi_t07_sim')
    parser.add_argument('--episodes', default='3,13',
                        help='held-out episode indices, comma separated')
    parser.add_argument('--samples', type=int, default=64)
    parser.add_argument('--batch-size', type=int, default=8)
    parser.add_argument('--seed', type=int, default=20261123)
    parser.add_argument('--out', type=Path)
    arguments = parser.parse_args()

    episodes = [int(v) for v in arguments.episodes.split(',') if v.strip()]
    started = time.perf_counter()

    policy, pre, post = load_policy(str(arguments.checkpoint))
    chunk_size = int(policy.config.chunk_size)
    dataset = build_dataset(str(arguments.dataset_root), arguments.repo_id,
                            episodes, chunk_size, fps=10.0)

    print('checkpoint   %s' % arguments.checkpoint)
    print('dataset      %d frames over episodes %s' % (len(dataset), episodes))
    print('chunk        %d steps @ 10 fps = %.2f s' % (chunk_size, chunk_size / 10.0))

    loader = DataLoader(dataset, batch_size=arguments.batch_size,
                        shuffle=False, num_workers=0)

    aggregates = []
    consumed = 0
    for batch in loader:
        result = score(policy, pre, post, batch, chunk_size, arguments.seed)
        aggregates.append(result)
        consumed += int(batch['action'].shape[0])
        if consumed >= arguments.samples:
            break

    def mean(key):
        values = [a[key] for a in aggregates if a[key] is not None]
        return sum(values) / len(values) if values else None

    per_joint = {}
    for name in JOINT_NAMES[:len(aggregates[0]['per_joint_raw'])]:
        values = [next(j for j in a['per_joint_raw'] if j['joint'] == name)
                  for a in aggregates]
        per_joint[name] = {
            'policy_l1_raw': round(sum(v['policy_l1_raw'] for v in values) / len(values), 5),
            'hold_l1_raw': round(sum(v['hold_l1_raw'] for v in values) / len(values), 5),
        }

    report = {
        'schema': 'smolvla_offline_eval_v1',
        'checkpoint': str(arguments.checkpoint),
        'dataset_root': str(arguments.dataset_root),
        'held_out_episodes': episodes,
        'samples': consumed,
        'chunk_size': chunk_size,
        'normalized_l1': round(mean('normalized_l1'), 6),
        'hold_normalized_l1': round(mean('hold_normalized_l1'), 6),
        'ratio_to_hold': round(mean('normalized_l1') / mean('hold_normalized_l1'), 4),
        'beats_hold': mean('normalized_l1') < mean('hold_normalized_l1'),
        'per_joint': per_joint,
        'seconds': round(time.perf_counter() - started, 2),
        'scope': ('held-out offline replay only; no Gazebo rollout and no task '
                  'success rate. Beating the hold baseline shows the model predicts '
                  'motion, not that it completes the task.'),
    }

    print()
    print('samples            %d' % consumed)
    print('normalized L1      %.6f' % report['normalized_l1'])
    print('hold baseline L1   %.6f  (predict "no motion")' % report['hold_normalized_l1'])
    print('ratio to hold      %.4f  %s'
          % (report['ratio_to_hold'],
             '<-- beats staying put' if report['beats_hold'] else '<-- WORSE than staying put'))
    print()
    print('%-9s %14s %14s' % ('joint', 'policy L1', 'hold L1'))
    for name, values in per_joint.items():
        print('%-9s %14.5f %14.5f'
              % (name, values['policy_l1_raw'], values['hold_l1_raw']))
    print()
    print(report['scope'])

    if arguments.out:
        arguments.out.parent.mkdir(parents=True, exist_ok=True)
        arguments.out.write_text(json.dumps(report, indent=2) + '\n')
        print('wrote %s' % arguments.out)


if __name__ == '__main__':
    main()
