import numpy as np

from imu_estimation.config import StationaryConfig
from imu_estimation.stationary import STANDARD_GRAVITY_MPS2, StationaryDetector
from imu_estimation.types import ImuSample, ImuStatus, MotionState, SAMPLE_STATUS_VALID


def _sample(sequence, gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, STANDARD_GRAVITY_MPS2)):
    return ImuSample(sequence=sequence, specific_force_body_mps2=accel,
                     gyro_pre_body_rad_s=gyro, status=SAMPLE_STATUS_VALID)


def test_non_overlapping_windows_require_hysteretic_entries():
    detector = StationaryDetector(StationaryConfig(window_samples=2, enter_static_windows=2, exit_static_windows=1))
    for sequence in (1, 2, 3):
        status, result = detector.update(_sample(sequence), (0.0, 0.0, 0.0), False, True)
        assert status == ImuStatus.OK
    assert result.window_ready is False
    status, result = detector.update(_sample(4), (0.0, 0.0, 0.0), False, True)
    assert status == ImuStatus.OK and result.window_ready and result.candidate_static
    assert result.state == MotionState.ANGULAR_STATIC
    assert result.sample_count == 2 and result.sequence == 4


def test_moving_window_exits_static_and_welford_statistics_are_sample_based():
    detector = StationaryDetector(StationaryConfig(window_samples=2, enter_static_windows=1, exit_static_windows=1))
    detector.update(_sample(1), (0.0, 0.0, 0.0), False, True)
    _, entered = detector.update(_sample(2), (0.0, 0.0, 0.0), False, True)
    assert entered.state == MotionState.ANGULAR_STATIC
    detector.update(_sample(3, gyro=(1.0, 0.0, 0.0)), (0.0, 0.0, 0.0), False, True)
    _, result = detector.update(_sample(4, gyro=(3.0, 0.0, 0.0)), (0.0, 0.0, 0.0), False, True)
    assert not result.candidate_static and result.state == MotionState.MOVING
    assert np.allclose(result.gyro_pre_mean_rad_s, (2.0, 0.0, 0.0))
    assert np.allclose(result.gyro_pre_std_rad_s, (2.0 ** 0.5, 0.0, 0.0))


def test_runtime_corrected_gyro_gate_is_not_applied_at_startup():
    config = StationaryConfig(window_samples=2, enter_static_windows=1, gyro_pre_mean_max_rad_s=1.0,
                              gyro_pre_std_max_rad_s=1.0, gyro_pre_peak_to_peak_max_rad_s=1.0)
    startup = StationaryDetector(config)
    startup.update(_sample(1, gyro=(0.1, 0.0, 0.0)), (0.0, 0.0, 0.0), True, True)
    _, start_result = startup.update(_sample(2, gyro=(0.1, 0.0, 0.0)), (0.0, 0.0, 0.0), True, True)
    assert start_result.candidate_static
    runtime = StationaryDetector(config)
    runtime.update(_sample(1, gyro=(0.1, 0.0, 0.0)), (0.0, 0.0, 0.0), True, False)
    _, run_result = runtime.update(_sample(2, gyro=(0.1, 0.0, 0.0)), (0.0, 0.0, 0.0), True, False)
    assert not run_result.candidate_static
