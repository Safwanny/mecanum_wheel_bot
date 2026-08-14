from pathlib import Path
import subprocess


REPOSITORY = Path(__file__).resolve().parents[2]


def check_ignored(relative_path):
    result = subprocess.run(
        ['git', 'check-ignore', '--quiet', str(relative_path)],
        cwd=REPOSITORY,
        check=False,
    )
    return result.returncode == 0


def test_runtime_evidence_roots_are_ignored():
    ignored = (
        'phase1_results/README.md',
        'phase1_results/manifest.json',
        'phase1_results/diagnostics/process_snapshot.json',
        'phase1_results/smoke_test/summary.json',
        'phase1_results/fused_comparison/trajectories/'
        'forward_1m_run_1.csv',
        'phase1_results/raw_baseline/plots/forward_1m_run_1.png',
        'phase1_results/formal_campaign/raw_only/forward_1m/'
        'launch_attempt_1.log',
        'odometry_results/summary.json',
    )
    assert all(check_ignored(path) for path in ignored)


def test_source_configuration_is_not_ignored():
    source = (
        'mobile_base_tools/config/odometry_tests.yaml',
        'mobile_base_tools/config/timestamp_contracts.yaml',
        'mobile_base_bringup/config/controllers.yaml',
        'mobile_base_localization/config/ekf.yaml',
    )
    assert all(not check_ignored(path) for path in source)
