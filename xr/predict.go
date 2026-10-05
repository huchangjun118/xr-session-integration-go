package xr

// PredictConfig 控制姿态预测与插值。
type PredictConfig struct {
	// Enabled 为 false 时输出退化为未经处理的原始姿态。
	Enabled bool
	// Horizon 是最大预测步长（秒），预测不得让输出倒退。
	Horizon float64
	// RecoveryBlend 是追踪恢复后的平滑时长（秒），避免姿态跳变。
	RecoveryBlend float64
}

// predictor 对单台设备做确定性姿态预测。
type predictor struct {
	cfg PredictConfig
}

func newPredictor(cfg PredictConfig) *predictor { return &predictor{cfg: cfg} }

// predict 基于最新采样外推到目标时间。
// 规则：
//   - 预测关闭时原样返回采样（未经处理的原始姿态）；
//   - targetTime 早于采样时间时不倒退，返回采样本身；
//   - 步长被 Horizon 截断，输出时间戳与实际所用步长一致。
func (p *predictor) predict(sample TimedPose, targetTime int64) TimedPose {
	if !p.cfg.Enabled {
		return sample
	}
	dt := float64(targetTime-sample.Time) / 1e9
	if dt <= 0 {
		return sample
	}
	if dt > p.cfg.Horizon {
		dt = p.cfg.Horizon
	}
	return TimedPose{
		Pose:     sample.Integrate(dt),
		Velocity: sample.Velocity,
		Time:     sample.Time + int64(dt*1e9),
	}
}
