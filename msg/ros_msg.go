package msg

import (
	"encoding/json"
	"math"
	"time"
)

type ROS_header struct {
	Seq      uint32    `json:"seq"`
	Stamp    TimeStamp `json:"stamp"`
	Frame_id string    `json:"frame_id"`
}

type TimeStamp struct {
	Secs  uint32 `json:"secs"`
	Nsecs uint32 `json:"nsecs"`
}

func (t TimeStamp) Time() time.Time {
	o := time.Unix(int64(t.Secs), int64(t.Nsecs))
	return o
}

func (t TimeStamp) Float64() float64 {
	return float64(t.Secs) + float64(t.Nsecs*uint32(math.Pow10(-9)))
}

func CalcTimeUnix(uni float64) time.Time {
	sec, dec := math.Modf(uni)
	t := time.Unix(int64(sec), int64(dec*1e9))
	return t
}

func FtoStamp(f float64) TimeStamp {
	sec, dec := math.Modf(f)
	t := TimeStamp{
		Secs:  uint32(sec),
		Nsecs: uint32(dec * 1e9),
	}
	return t
}

func CalcStamp(t time.Time) TimeStamp {
	o := TimeStamp{
		Secs:  uint32(t.Unix()),
		Nsecs: uint32(t.UnixNano()),
	}
	return o
}

type Point struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

func (p Point) Distance(o Point) float64 {
	return math.Hypot(p.X-o.X, p.Y-o.Y)
}

type Quaternion struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
	W float64 `json:"w"`
}

type Pose struct {
	Position    Point      `json:"position"`
	Orientation Quaternion `json:"orientation"`
}

type ROS_PoseStamped struct {
	Header ROS_header `json:"header"`
	Pose   Pose       `json:"pose"`
}

type Path struct {
	Header ROS_header        `json:"header"`
	Poses  []ROS_PoseStamped `json:"poses"`
}

func MakePathMsg(route [][3]float64) ([]byte, error) {
	var poses []ROS_PoseStamped

	for i := 1; i < len(route); i++ {
		x := route[i][1]
		y := route[i][2]
		prevX := route[i-1][1]
		prevY := route[i-1][2]

		yaw := math.Atan2((y - prevY), (x - prevX))

		pos := ROS_PoseStamped{
			Header: ROS_header{
				Seq:      uint32(i),
				Stamp:    FtoStamp(route[i][0]),
				Frame_id: "map",
			},
			Pose: Pose{
				Position:    Point{X: x, Y: y, Z: 0.0},
				Orientation: Yaw2Quaternion(yaw),
			},
		}
		poses = append(poses, pos)
	}

	planm := Path{
		Header: ROS_header{Frame_id: "map"},
		Poses:  poses,
	}

	jm, err := json.MarshalIndent(planm, "", " ")
	if err != nil {
		return jm, err
	}
	return jm, err

}

func MakePathMsg2D(route [][2]float64) ([]byte, error) {
	var poses []ROS_PoseStamped

	for i := 0; i < len(route); i++ {
		x := route[i][0]
		y := route[i][1]
		pos := ROS_PoseStamped{
			Header: ROS_header{Seq: uint32(i)},
			Pose: Pose{
				Position: Point{X: x, Y: y, Z: 0.0},
			},
		}
		poses = append(poses, pos)
	}

	planm := Path{
		Header: ROS_header{Frame_id: "map"},
		Poses:  poses,
	}

	jm, err := json.MarshalIndent(planm, "", " ")
	if err != nil {
		return jm, err
	}
	return jm, err

}

func Eular2Quaternion(yaw, pitch, roll float64) Quaternion {
	cy := math.Cos(yaw * 0.5)
	sy := math.Sin(yaw * 0.5)
	cp := math.Cos(pitch * 0.5)
	sp := math.Sin(pitch * 0.5)
	cr := math.Cos(roll * 0.5)
	sr := math.Sin(roll * 0.5)

	var q Quaternion
	q.W = cr*cp*cy + sr*sp*sy
	q.X = sr*cp*cy - cr*sp*sy
	q.Y = cr*sp*cy + sr*cp*sy
	q.Z = cr*cp*sy - sr*sp*cy
	return q
}

func Yaw2Quaternion(yaw float64) Quaternion {
	cy := math.Cos(yaw / 2)
	sy := math.Sin(yaw / 2)

	var q Quaternion
	q.W = cy
	q.X = 0
	q.Y = 0
	q.Z = sy
	return q
}
