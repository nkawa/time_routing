package routing

import (
	"fmt"
	"log"
	"math"
	"sync"
	"time"
)

var (
	Mu sync.Mutex
)

const (
	// Mode = 1 //around 6
	Mode = 2 //around 12
)

// initialize custom resolution
// using main
func NewGridMapResoHexa(m MapMeta, robotRadius float64, resolution float64, objMap [][2]float64) *GridMap {
	start := time.Now()
	g := new(GridMap)
	g.MapOrigin = m.Origin
	g.Resolution = resolution

	worldWidth := float64(m.W) * m.Reso
	worldHeight := float64(m.H) * m.Reso

	width := int(math.Ceil(worldWidth / (resolution * math.Sqrt(3) / 2)))
	height := int(math.Ceil(worldHeight/resolution)) + 1

	g.Origin.X = m.Origin.X
	g.Origin.Y = m.Origin.Y

	g.Width = width
	g.Height = height

	g.MaxT = MaxTimeLength
	count := 0

	g.ObjectMap = make(map[Index]bool)
	for i := 0; i < width; i++ {
		x := m.Origin.X + float64(i)*resolution*math.Sqrt(3)/2
		for j := 0; j < height; j++ {
			var y float64
			if i%2 == 0 {
				y = m.Origin.Y + float64(j)*resolution
			} else {
				y = m.Origin.Y - m.Reso/2 + float64(j)*resolution
			}
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

func (g *GridMap) Pos2IndHexa(x, y float64) (int, int) {
	if x < g.MapOrigin.X || y < g.MapOrigin.Y {
		return 0, 0
	}
	aid := int(math.Round((x - g.Origin.X) / (g.Resolution * math.Sqrt(3) / 2)))
	var bid int
	if aid%2 == 0 {
		bid = int(math.Round((y - g.Origin.Y) / g.Resolution))
	} else {
		bid = int(math.Round((y - g.Origin.Y - g.Resolution/2) / g.Resolution))
	}
	return aid, bid
}

func (g *GridMap) Ind2PosHexa(a, b int) (float64, float64) {
	if a < 0 || b < 0 {
		return 0, 0
	}
	x := g.Origin.X + (g.Resolution*math.Sqrt(3)/2)*float64(a)
	var y float64
	if a%2 == 0 {
		y = g.Origin.Y + g.Resolution*float64(b)
	} else {
		y = g.Origin.Y - g.Resolution/2 + g.Resolution*float64(b)
	}
	return x, y
}

type logOpt struct {
	X         int
	Y         int
	T         int
	Cost      float64
	StopCount int
}

func (m GridMap) PlanHexa(id int, sa, sb, ga, gb int, v, timeStep float64, TRW TimeRobotMap, otherObjects map[Index]bool, timeBeta float64) (route [][3]int, oerr error) {
	startTime := time.Now()

	if m.ObjectMap[newIndex(ga, gb)] {
		oerr = fmt.Errorf("robot%d path planning error: goal is not verified", id)
		return nil, oerr
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
			return nil, oerr
		}

		// 30秒以上で諦める
		// if time.Since(startTime).Seconds() > 30 {
		// 	a := m.Origin.X + float64(current.XId)*m.Resolution
		// 	b := m.Origin.Y + float64(current.YId)*m.Resolution
		// 	log.Printf("path planning time out. count %d current is (%d, %f, %f)",
		// 		count,
		// 		current.T,
		// 		getXAB(a, b),
		// 		getYAB(a, b),
		// 	)
		// 	bytes, jerr := json.MarshalIndent(logData, "", " ")
		// 	if jerr != nil {
		// 		log.Print(jerr)
		// 	}
		// 	now := time.Now()
		// 	ioutil.WriteFile(fmt.Sprintf("log/route/%s/fail_route%d_%s.log", now.Format("2006-01-02"), id, now.Format("01-02-15-4")), bytes, 0666)
		// 	oerr = errors.New("path planning timeouted")
		// 	routei := m.finalPath(goal, closeSetT)
		// 	return routei, oerr
		// }

		// get minimum cost node in open set
		minCost := 9999999999999999999.9
		minTime = 999999999999999999
		var minKey IndexT
		for key, val := range openSetT {
			calCost := val.G + val.S + m.heuristicHexa(goal, val) // length to goal
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
				elaps,
				count,
				len(route),
			)
			log.Printf("it has %d length, %f seconds", len(route), float64(len(route))*timeStep)
			return route, nil
		}

		delete(openSetT, minKey)
		closeSet[nodeIndex(current)] = current
		closeSetT[nodeIndexT(current)] = current

		around := current.AroundHexa(&m, minTime, v, timeStep, TRW, otherObjects, timeBeta)
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

func (g GridMap) Route2PosHexa(minT time.Time, timeStep float64, route [][3]int) ([]time.Time, [][2]float64) {
	l := len(route)
	fRoute := make([][2]float64, l)
	fTime := make([]time.Time, l)

	for i, r := range route {
		x, y := g.Ind2PosHexa(r[1], r[2])
		t := minT.Add(time.Duration(float64(r[0])*timeStep) * time.Second)
		p := [2]float64{x, y}
		fRoute[i] = p
		fTime[i] = t
	}
	return fTime, fRoute
}

func (n Node) AroundHexa(g *GridMap, minTime int, v, timeStep float64, TRW TimeRobotMap, otherRobot map[Index]bool, timeBeta float64) []*Node {
	// [time, x, y, cost]
	motion := [][4]float64{
		{1.0, 0.0, 0.0, 0.0},
		{0.0, 1.0, 0.0, 1.0},
		{0.0, 1.0, 1.0, 1.0},
		{0.0, 0.0, 1.0, 1.0},
		{0.0, -1.0, 1.0, 1.0},
		{0.0, -1.0, 0.0, 1.0},
		{0.0, 0.0, -1.0, 1.0},
	}
	if Mode == 2 {
		r3 := math.Sqrt(3)
		motion2 := [][4]float64{
			//1個奥
			{1.0, 2.0, 0.0, r3},
			{1.0, 1.0, 2.0, r3},
			{1.0, -1.0, 2.0, r3},
			{1.0, -2.0, 0.0, r3},
			{1.0, -1.0, -1.0, r3},
			{1.0, 1.0, -1.0, r3},
		}
		motion = append(motion, motion2...)
	}

	var around []*Node
	for _, m := range motion {
		aX := n.XId + int(m[1])
		aY := n.YId + int(m[2])
		aT := n.T + int(m[0]) + 1

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

		node := n.NewNode(aT, aX, aY, n.G+m[3], n.S+timeBeta*m[0])

		// MaxStopCount以上止まりすぎはだめ
		// if node.S > MaxS {
		// 	continue
		// }

		around = append(around, node)
	}
	return around
}

func (g GridMap) heuristicHexa(n1, n2 *Node) float64 {
	w := 1.0

	x1, y1 := g.Ind2PosHexa(n1.XId, n1.YId)
	x2, y2 := g.Ind2PosHexa(n2.XId, n2.YId)
	d := w * math.Hypot(x1-x2, y1-y2)
	return d
}

func sumHexa(n1, n2 *Node) float64 {
	w := 1.0
	d := w*math.Abs(float64(n1.XId)-float64(n2.XId)) + math.Abs(float64(n1.YId)-float64(n2.YId))
	return d
}
