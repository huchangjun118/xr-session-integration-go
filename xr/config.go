package xr

// Config 是集成层的整体配置。
type Config struct {
	Predict PredictConfig
	// Version 标识算法/配置版本，用于复现性判定。
	Version string
}

// DefaultConfig 返回默认配置。
func DefaultConfig() Config {
	return Config{
		Predict: PredictConfig{
			Enabled:       true,
			Horizon:       0.05,
			RecoveryBlend: 0.1,
		},
		Version: "v1",
	}
}

// CompatibleWith 判定两份配置下相同输入的结果是否可复现：
// 只有版本与全部配置项一致时才保证逐条一致；
// 否则明确判定为不可复现，调用方不得混用结果。
func (c Config) CompatibleWith(other Config) bool {
	return c.Version == other.Version &&
		c.Predict.Enabled == other.Predict.Enabled &&
		c.Predict.Horizon == other.Predict.Horizon &&
		c.Predict.RecoveryBlend == other.Predict.RecoveryBlend
}
