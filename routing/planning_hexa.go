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
	// Mode = 1 //search around 6 node
	Mode = 2 //search around 12 node
)

func (m GridMap) PlanHexa(id int, sa, sb, ga, gb int, v, timeStep float64, TRW TimeRobotMap, otherObjects map[Index]bool, timeBeta float64, is2d bool) (route [][3]int, oerr error) {
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

		around := current.AroundHexa(&m, minTime, v, timeStep, TRW, otherObjects, timeBeta, is2d)
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

func (n Node) AroundHexa(g *GridMap, minTime int, v, timeStep float64, TRW TimeRobotMap, otherRobot map[Index]bool, timeBeta float64, is2d bool) []*Node {
	// [time, x, y, cost]
	motion := [][4]float64{
		{1.0, 0.0, 0.0, 0.0}, //stop
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
	for i, m := range motion {
		if is2d && i == 0 {
			continue
		}
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
