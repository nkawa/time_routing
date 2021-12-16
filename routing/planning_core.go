package routing

import (
	"errors"

	"log"
	"math"
	"time"
)

type Node struct {
	T   int `json:"t"` //time
	XId int `json:"x"`
	YId int `json:"y"`

	G float64 `json:"g"` // step cost
	F float64 `json:"f"` // cost
	S float64 `json:"s"` // stop cost

	Parent *Node
}

type Index int64

const HASH int64 = 100000
const HASH2 int64 = HASH * HASH

func newIndex(x, y int) Index {
	n := int64(x) + int64(y)*HASH
	return Index(n)
}

func NewIndex(x, y int) Index {
	n := int64(x) + int64(y)*HASH
	return Index(n)
}

func nodeIndex(n *Node) Index {
	i := Index(int64(n.XId) + int64(n.YId)*HASH)
	return i
}

type IndexT int64

func nodeIndexT(n *Node) IndexT {
	i := IndexT(int64(n.XId) + int64(n.YId)*HASH + int64(n.T)*HASH2)
	return i
}

func newIndexT(t, x, y int) IndexT {
	n := IndexT(int64(x) + int64(y)*HASH + int64(t)*HASH2)
	return n
}

func NewIndexT(t, x, y int) IndexT {
	n := IndexT(int64(x) + int64(y)*HASH + int64(t)*HASH2)
	return n
}

func (m GridMap) Plan(sx, sy, gx, gy int, TRW TimeRobotMap) (route [][3]int, oerr error) {
	startTime := time.Now()

	if m.ObjectMap[newIndex(gx, gy)] {
		oerr = errors.New("path planning error: goal is not verified")
		return nil, oerr
	}
	if m.ObjectMap[newIndex(sx, sy)] {
		oerr = errors.New("path planning error: start point is not verified")
		return nil, oerr
	}

	start := &Node{T: 0, XId: sx, YId: sy, G: 0, Parent: nil}
	goal := &Node{T: 0, XId: gx, YId: gy, G: 0, Parent: nil}

	openSet := make(map[IndexT]*Node)

	closeSet := make(map[Index]*Node)
	closeSetT := make(map[IndexT]*Node)

	openSet[nodeIndexT(start)] = start

	count := 0
	current := &Node{T: 0}
	var minTime int

	for {
		count += 1
		if len(openSet) == 0 {
			elaps := time.Since(startTime).Seconds()
			log.Print(current.T, current.XId, current.YId, count, elaps)
			oerr = errors.New("path planning error: open set is empty")
			return nil, oerr
		}

		// get min cost node in open set
		minCost := 9999999999999999999.9
		minTime = 99999999999999999
		var minKey IndexT
		for key, val := range openSet {
			calCost := val.G + val.S + heuristic(goal, val)
			if calCost < minCost {
				minCost = calCost
				minKey = key
			}
			if val.T < minTime {
				minTime = val.T
			}
		}
		current = openSet[minKey]

		if current.XId == gx && current.YId == gy {
			log.Print("find goal")
			goal.Parent = current.Parent
			goal.F = current.F
			goal.T = current.T
			elaps := time.Since(startTime).Seconds()
			log.Printf("planning (%f, %f) to (%f, %f) takes %f seconds, count is %d",
				m.Origin.X+float64(sx)*m.Resolution,
				m.Origin.Y+float64(sy)*m.Resolution,
				m.Origin.X+float64(gx)*m.Resolution,
				m.Origin.Y+float64(gy)*m.Resolution,
				elaps,
				count,
			)
			routei := m.finalPath(goal, closeSetT)
			return routei, nil
		}

		delete(openSet, minKey)
		closeSet[nodeIndex(current)] = current
		closeSetT[nodeIndexT(current)] = current

		around := current.Around(&m, minTime, TRW)
		for _, an := range around {
			indT := nodeIndexT(an)
			ind := nodeIndex(an)

			if _, ok := closeSet[ind]; ok {
				if an.Parent.XId != an.XId || an.Parent.YId != an.YId { // not allow to passing same node other than stop
					continue
				}
			}

			if _, ok := openSet[indT]; !ok {
				openSet[indT] = an
			}
		}
	}
}

func (m GridMap) finalPath(goal *Node, closeSet map[IndexT]*Node) (route [][3]int) {
	route = append(route, [3]int{goal.T, goal.XId, goal.YId})

	parent := goal.Parent
	for parent != nil {
		n := closeSet[nodeIndexT(parent)]
		route = append([][3]int{{n.T, n.XId, n.YId}}, route...)
		parent = n.Parent
	}
	return route
}

func (g GridMap) Route2Pos(minT float64, route [][3]int) [][3]float64 {
	l := len(route)
	fRoute := make([][3]float64, l)

	for i, r := range route {
		x, y := g.Ind2Pos(r[1], r[2])
		t := minT + float64(r[0]*i)
		p := [3]float64{t, x, y}
		fRoute[i] = p
	}
	return fRoute
}

func (p *Node) NewNode(t, x, y int, g, s float64) *Node {
	n := new(Node)
	n.T = t
	n.XId = x
	n.YId = y
	n.Parent = p
	n.G = g
	n.S = s

	return n
}

func (n *Node) Around(g *GridMap, minTime int, TRW TimeRobotMap) []*Node {
	// time, x, y, cost
	motion := [9][4]float64{
		{1.0, 0.0, 0.0, 0.0}, //stay 要修正
		{0.0, 1.0, 0.0, 1.0},
		{0.0, 0.0, 1.0, 1.0},
		{0.0, -1.0, 0.0, 1.0},
		{0.0, 0.0, -1.0, 1.0},
		{0.0, -1.0, -1.0, math.Sqrt(2)},
		{0.0, -1.0, 1.0, math.Sqrt(2)},
		{0.0, 1.0, -1.0, math.Sqrt(2)},
		{0.0, 1.0, 1.0, math.Sqrt(2)},
	}
	var around []*Node
	for _, m := range motion {
		aX := n.XId + int(m[1])
		aY := n.YId + int(m[2])
		aT := n.T + 1

		if aT >= g.MaxT {
			continue
		}

		//map外のノードは外す
		if aX < 0 || aX >= g.Width {
			continue
		}
		if aY < 0 || aY >= g.Height {
			continue
		}

		//元から障害物で通れないところは消す
		if g.ObjectMap[newIndex(aX, aY)] {
			continue
		}

		//ロボットがいて通れないコストマップは外す
		if TRW[newIndexT(aT, aX, aY)] {
			continue
		}

		node := n.NewNode(aT, aX, aY, n.G+m[3], n.S+m[0])
		around = append(around, node)
	}
	return around
}

func heuristic(n1, n2 *Node) float64 {
	// n1 is goal
	w := 1.0
	d := w * math.Hypot(float64(n1.XId)-float64(n2.XId), float64(n1.YId)-float64(n2.YId))
	// d := w * math.Sqrt(math.Pow(float64(n1.XId)-float64(n2.XId), 2)+math.Pow(float64(n1.YId)-float64(n2.YId), 2)+math.Pow(n2.S, 2))
	return d
}
