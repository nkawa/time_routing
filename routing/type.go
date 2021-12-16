package routing

import "sort"

var (
	MaxTimeLength int = 100000 //これ以上のtを計算しない
)

type logOpt struct {
	X         int
	Y         int
	T         int
	Cost      float64
	StopCount int
}

type Point struct {
	X float64
	Y float64
}

// type CostMap map[Index]uint8
type TimeRobotMap map[IndexT]bool
type ObjMap map[Index]bool

func MinFloat(a []float64) float64 {
	sort.Float64s(a)
	return a[0]
}

func MaxFloat(a []float64) float64 {
	sort.Float64s(a)
	return a[len(a)-1]
}

func NewTRW(t, w, h int) TimeRobotMap {
	var tw TimeRobotMap = make(map[IndexT]bool)
	return tw
}
