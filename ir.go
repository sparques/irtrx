package irtrx

import "time"

const (
	// Freq38Khz is the most commonly used frequency for IR remotes
	Freq38Khz = 38000
)

// TimePair encodes two durations used to encode an on-off or off-on amount of time.
type TimePair [2]time.Duration

// FrameMarshaller defines an interface for marshalling data to slice of TimePairs
type FrameMarshaller interface {
	MarshalFrame() []TimePair
}
