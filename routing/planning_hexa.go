package routing

import (
	"encoding/json"
	"errors"
	"fmt"
	"io/ioutil"
	"log"
	"math"
	"sync"
	"time"
)

var (
	Mu sync.Mutex
)

const (
	Mode = 1 //around 6
	// Mode = 2 //around 12
)

// initialize custom resolution
// using main
func NewGridMapResoHexa(m MapMeta, robotRadius float64, resolution float64, objMap [][2]float64) *GridMap {
	start := time.Now()
	g := new(GridMap)
	g.MapOrigin = m.Origin
	g.Resolution = resolution

	var aList []float64
	var bList []float64

	for _, obj := range objMap {
		aList = append(aList, getA(obj[0], obj[1]))
		bList = append(bList, getB(obj[0], obj[1]))
	}

	maxX := MaxFloat(aList)
	maxY := MaxFloat(bList)
	g.Origin.X = MinFloat(aList)
	g.Origin.Y = MinFloat(bList)

	g.Width = int(math.Round((maxX - g.Origin.X) / resolution))
	g.Height = int(math.Round((maxY - g.Origin.Y) / resolution))

	g.MaxT = MaxTimeLength
	g.ObjectMap = make(map[Index]bool, g.Height)

	count := 0
	for j := 0; j < g.Height; j++ {
		b := g.Origin.Y + float64(j)*g.Resolution
		for i := 0; i < g.Width; i++ {
			a := g.Origin.X + float64(i)*g.Resolution
			x := getXAB(a, b)
			y := getYAB(a, b)
			g.ObjectMap[newIndex(i, j)] = false
			for _, op := range objMap {
				d := math.Hypot(op[0]-x, op[1]-y)
				if d <= robotRadius {
					g.ObjectMap[newIndex(i, j)] = true
					count += 1
					break
				}
			}
		}
	}

	elaps := time.Since(start).Seconds()
	log.Printf("loading gridmap resolution: %f, takes: %f seconds, obj %d counts, width: %d, height: %d, origin %f,%f", resolution, elaps, count, g.Width, g.Height, g.Origin.X, g.Origin.Y)
	return g
}

func getA(x, y float64) float64 {
	return x/math.Sqrt(3) + y
}

func getB(x, y float64) float64 {
	return x/math.Sqrt(3) - y
}

func A(x, y float64) float64 {
	return x/math.Sqrt(3) + y
}

func B(x, y float64) float64 {
	return x/math.Sqrt(3) - y
}

func getXAB(a, b float64) float64 {
	return math.Sqrt(3)/2*a + math.Sqrt(3)/2*b
}

func getYAB(a, b float64) float64 {
	return a/2 - b/2
}

func (g GridMap) Pos2IndHexa(x, y float64) (int, int) {
	if x < g.MapOrigin.X || y < g.MapOrigin.Y {
		log.Printf("position (%f,%f) is out of map", x, y)
		return 0, 0
	}
	a := getA(x, y)
	b := getB(x, y)
	xid := int(math.Round((a - g.Origin.X) / g.Resolution))
	yid := int(math.Round((b - g.Origin.Y) / g.Resolution))
	return xid, yid
}

type logOpt struct {
	X         int
	Y         int
	T         int
	Cost      float64
	StopCount int
}

func (m GridMap) PlanHexa(id int, sa, sb, ga, gb int, v, w, timeStep float64, TRW TimeRobotMap, otherRobot map[Index]bool) (route [][3]int, stops []int, oerr error) {
	startTime := time.Now()
	//var logData []logOpt

	sx := int(getXAB(float64(sa), float64(sb)))
	sy := int(getYAB(float64(sa), float64(sb)))
	gx := int(getXAB(float64(ga), float64(gb)))
	gy := int(getYAB(float64(ga), float64(gb)))

	log.Printf("start planning robot%d (%f, %f) to (%f, %f)",
		id,
		m.MapOrigin.X+float64(sx)*m.Resolution,
		m.MapOrigin.Y+float64(sy)*m.Resolution,
		m.MapOrigin.X+float64(gx)*m.Resolution,
		m.MapOrigin.Y+float64(gy)*m.Resolution,
	)

	if m.ObjectMap[newIndex(ga, gb)] {
		oerr = fmt.Errorf("robot%d path planning error: goal is not verified", id)
		return nil, nil, oerr
	}

	// init start,goal nodes
	start := &Node{T: 0, XId: sa, YId: sb, G: 0, Parent: nil}
	goal := &Node{T: 0, XId: ga, YId: gb, G: 0, Parent: nil}

	openSetT := make(map[IndexT]*Node) // once passed node

	closeSet := make(map[Index]*Node)
	closeSetT := make(map[IndexT]*Node)

	openSetT[nodeIndexT(start)] = start

	count := 0
	current := &Node{T: 0}
	var minTime int
	var logData []logOpt

	for {
		count += 1
		// failure
		if len(openSetT) == 0 {
			oerr = fmt.Errorf("path planning error: open set is empty, count %d", count)
			return nil, nil, oerr
		}

		// 30秒以上で諦める
		if time.Since(startTime).Seconds() > 30 {
			a := m.Origin.X + float64(current.XId)*m.Resolution
			b := m.Origin.Y + float64(current.YId)*m.Resolution
			log.Printf("path planning time out. count %d current is (%d, %f, %f)",
				count,
				current.T,
				getXAB(a, b),
				getYAB(a, b),
			)
			bytes, jerr := json.MarshalIndent(logData, "", " ")
			if jerr != nil {
				log.Print(jerr)
			}
			now := time.Now()
			ioutil.WriteFile(fmt.Sprintf("log/route/%s/fail_route%d_%s.log", now.Format("2006-01-02"), id, now.Format("01-02-15-4")), bytes, 0666)
			oerr = errors.New("path planning timeouted")
			routei := m.finalPath(goal, closeSetT)
			return routei, nil, oerr
		}

		// get minimum cost node in open set
		minCost := 9999999999999999999.9
		minTime = 999999999999999999
		var minKey IndexT
		for key, val := range openSetT {
			calCost := val.G + val.S + heuristicHexa(goal, val) // length to goal
			if calCost < minCost {
				minCost = calCost
				minKey = key
			}
			if val.T < minTime {
				minTime = val.T
			}
		}
		current = openSetT[minKey]
		if current == nil {
			log.Printf("Error, current is nil. openset is %d minKey is %d, cost is %f", len(openSetT), minKey, minCost)
		}

		logData = append(logData, logOpt{current.XId, current.YId, current.T, current.F, int(current.S)})

		// find Goal
		if current.XId == ga && current.YId == gb {
			log.Print("find goal")

			goal.Parent = current.Parent
			goal.F = current.F
			goal.T = current.T
			elaps := time.Since(startTime).Seconds()
			route := m.finalPath(goal, closeSetT)
			log.Printf("robot%d planning took %f seconds, count: %d, length: %d",
				id,
				// m.MapOrigin.X+float64(sx)*m.Resolution,
				// m.MapOrigin.Y+float64(sy)*m.Resolution,
				// m.MapOrigin.X+float64(gx)*m.Resolution,
				// m.MapOrigin.Y+float64(gy)*m.Resolution,
				elaps,
				count,
				len(route),
			)
			log.Printf("it has %d length, %f seconds", len(route), float64(len(route))*timeStep)
			return route, stops, nil
		}

		delete(openSetT, minKey)
		closeSet[nodeIndex(current)] = current
		closeSetT[nodeIndexT(current)] = current

		around := current.AroundHexa(&m, minTime, v, w, timeStep, TRW, otherRobot)
		for _, an := range around {
			indT := nodeIndexT(an)
			ind := nodeIndex(an)

			// すでに通った道を戻るのはだめ
			if _, ok := closeSet[ind]; ok {
				if an.Parent.XId != an.XId || an.Parent.YId != an.YId { // ただし、止まる場合を除く
					continue
				}
			}

			if _, ok := openSetT[indT]; !ok {
				openSetT[indT] = an
			}
		}
	}
}

func (g GridMap) Route2PosHexa(minT float64, timeStep float64, route [][3]int) [][3]float64 {
	l := len(route)
	fRoute := make([][3]float64, l)

	for i, r := range route {
		a, b := g.Ind2Pos(r[1], r[2])
		x := getXAB(a, b)
		y := getYAB(a, b)
		t := minT + float64(r[0])*timeStep
		p := [3]float64{t, x, y}
		fRoute[i] = p
	}
	return fRoute
}

func (n Node) AroundHexa(g *GridMap, minTime int, v, w, timeStep float64, TRW TimeRobotMap, otherRobot map[Index]bool) []*Node {
	// [time, x, y, cost]
	motion := [][4]float64{
		{1.0, 0.0, 0.0, 0.0},
		{0.0, 1.0, 0.0, 1.0},
		{0.0, 0.0, 1.0, 1.0},
		{0.0, -1.0, 1.0, 1.0},
		{0.0, -1.0, 0.0, 1.0},
		{0.0, 0.0, -1.0, 1.0},
		{0.0, 1.0, -1.0, 1.0},
	}
	if Mode == 2 {
		motion2 := [][4]float64{
			//1個奥
			{1.0, 1.0, 1.0, 2.0},
			{1.0, 2.0, -1.0, 2.0},
			{1.0, -1.0, -1.0, 2.0},
			{1.0, -2.0, 1.0, 2.0},
			{1.0, 1.0, -2.0, 2.0},
			{1.0, -1.0, 2.0, 2.0},
		}
		motion = append(motion, motion2...)
	}

	var around []*Node
	for _, m := range motion {
		aX := n.XId + int(m[1])
		aY := n.YId + int(m[2])
		aT := n.T + 1

		//時間コストマップ外の時間は外す
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

		//元から障害物で通れないところは外す
		if g.ObjectMap[newIndex(aX, aY)] {
			continue
		}

		//他のロボットがいる場所は外す
		if val, ok := otherRobot[newIndex(aX, aY)]; ok {
			if val {
				continue
			}
		}

		//移動中のロボットがいて通れないところは外す
		if val, ok := TRW[newIndexT(aT, aX, aY)]; ok {
			if val {
				continue
			}
		}

		node := n.NewNode(aT, aX, aY, n.G+m[3], n.S+0.1*m[0])

		// MaxStopCount以上止まりすぎはだめ
		if node.S > MaxS {
			continue
		}

		around = append(around, node)
	}
	return around
}

func heuristicHexa(n1, n2 *Node) float64 {
	w := 1.0
	// x1 := getXAB(float64(n1.XId), float64(n1.YId))
	// y1 := getYAB(float64(n1.XId), float64(n1.YId))
	// x2 := getXAB(float64(n2.XId), float64(n2.YId))
	// y2 := getYAB(float64(n2.XId), float64(n2.YId))
	// d := w * math.Hypot(x1-x2, y1-y2)

	d := w * math.Hypot(float64(n1.XId)-float64(n2.XId), float64(n1.YId)-float64(n2.YId))
	return d
}

func sumHexa(n1, n2 *Node) float64 {
	w := 1.0
	d := w*math.Abs(float64(n1.XId)-float64(n2.XId)) + math.Abs(float64(n1.YId)-float64(n2.YId))
	return d
}
