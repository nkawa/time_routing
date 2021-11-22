package robot

import (
	"encoding/json"
	"log"
	"time"

	ros "github.com/fukurin00/go_ros_msg"
)

type RobotStatus struct {
	Id     int
	Radius float64

	Path [][3]int
	Pos  ros.Point
	Dest ros.Point

	HavePath      bool
	LastPosUpdate time.Time
}

func NewRobot(id int, radius float64) *RobotStatus {
	r := new(RobotStatus)
	r.Id = id
	r.Radius = radius
	r.HavePath = false
	r.LastPosUpdate = time.Time{}
	return r
}

func (r *RobotStatus) SetPos(rcd []byte) {
	if time.Since(r.LastPosUpdate).Seconds() > 0.5 {
		var pos *ros.Pose
		err := json.Unmarshal(rcd, pos)
		if err != nil {
			log.Print("[Error/robot]cannot unmarshal pos message:"err)
		} else {
			r.Pos = pos.Position
			r.LastPosUpdate = time.Now()
			if r.HavePath {
				r.CheckDest()
			}
		}
	}

}

func (r *RobotStatus) SetPath(path [][3]int) {
	r.Path = path
	r.HavePath = true
}

func (r *RobotStatus) SetDest(dest ros.Point) {
	r.Dest = dest
}

func (r *RobotStatus) CheckDest() {
	if r.Dest.Distance(r.Pos) < 1 {
		r.HavePath = false
	}
}
