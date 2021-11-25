package routing

import "math"

var (
	Around     = [][2]int{{-1, 0}, {0, -1}, {1, -1}, {1, 0}, {0, 1}, {-1, 1}}
	AroundMore = [][2]int{{2, -1}, {1, -2}, {-1, -1}, {-2, 1}, {2, -1}, {1, 1}}
	Around2    = [][2]int{{-2, 0}, {0, -2}, {2, -2}, {2, 0}, {0, 2}, {-2, 2}}
	Arrond3    = [][2]int{{-3, 0}, {0, -3}, {3, -3}, {3, 0}, {0, 3}, {-3, 3}, {2, 1}, {1, 2}, {3, -1}, {3, -2}, {2, -3}, {1, -3}, {-1, -2}, {-2, -1}, {-3, 1}, {-3, 2}, {-2, 3}, {-1, 3}}
)

func GetAoundCell(r, l float64) int {
	// r: robotRadius
	// l: resolution
	if 2*1.1*r <= l/2 {
		return 1
	} else if 2*1.1*r <= 2/math.Sqrt(3)*l {
		return 2
	} else if 2*1.1*r <= 4/math.Sqrt(3)*l {
		return 3
	}
	//return 4

	return int(math.Ceil(math.Sqrt(3) * 1.1 * r / l))
}

func (g GridMap) UpdateStep(TW TimeRobotMap, step int) TimeRobotMap {
	newTRW := make(map[IndexT]bool)
	for key, val := range TW {
		t, x, y := key.GetXYT()
		if t >= step {
			newTRW[newIndexT(t-step, x, y)] = val
		}
	}
	return newTRW
}

func (key IndexT) GetXYT() (int, int, int) {
	t := int(key / IndexT(HASH2))
	x := int(key % IndexT(HASH2) % IndexT(HASH))
	y := int((key / IndexT(HASH)) % IndexT(HASH))
	return t, x, y
}

func (key Index) GetXY() (int, int) {
	x := int(key % Index(HASH))
	y := int(key / Index(HASH))
	return x, y
}

func TRWCopy(current TimeRobotMap) TimeRobotMap {
	trw := make(map[IndexT]bool)

	for key, val := range current {
		trw[key] = val
	}
	return trw
}

func (g GridMap) UpdateTimeObjMapHexa(TW TimeRobotMap, route [][3]int, aroundCell int, timeStepLoos int) {

	var ar [][2]int
	var it int
	var ix int
	var iy int
	for i := 0; i < len(route); i++ {
		for j := 0; j <= timeStepLoos; j++ {
			it = route[i][0] + j
			if it < 0 || it > MaxTimeLength {
				continue
			}
			ix = route[i][1]
			iy = route[i][2]
			//center
			TW[newIndexT(it, ix, iy)] = true
			//around
			if aroundCell == 2 {
				ar = append(ar, Around...)
			} else if aroundCell == 3 {
				ar = append(ar, Around...)
				ar = append(ar, Around2...)
				ar = append(ar, AroundMore...)
			} else {
				ar = append(ar, Around...)
				ar = append(ar, Around2...)
				ar = append(ar, AroundMore...)
				ar = append(ar, Arrond3...)
			}
			for _, v := range ar {
				ny := iy + v[1]
				nx := ix + v[0]
				if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
					continue
				}
				TW[newIndexT(it, nx, ny)] = true
			}
		}
	}
}

// func (g GridMap) UpdateTimeRobotMap(route [][3]int, robotRadius float64, TRW TimeRobotMap) {
// 	around8 := [8][2]int{{-1, 0}, {0, 1}, {1, 0}, {0, -1}, {-1, -1}, {-1, 1}, {1, 1}, {1, -1}}
// 	around4 := [4][2]int{{-1, 0}, {0, 1}, {1, 0}, {0, -1}}

// 	for i := 0; i < len(route); i++ {
// 		it := route[i][0]
// 		ix := route[i][1]
// 		iy := route[i][2]
// 		TRW[it][iy][ix] = true
// 		if robotRadius < g.Resolution {
// 			continue
// 		} else if robotRadius <= math.Sqrt(2)*g.Resolution {
// 			for _, v := range around4 {
// 				ny := iy + v[1]
// 				nx := ix + v[0]
// 				if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
// 					continue
// 				}
// 				TRW[it][ny][nx] = true
// 			}
// 		} else {
// 			for _, v := range around8 {
// 				ny := iy + v[1]
// 				nx := ix + v[0]
// 				if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
// 					continue
// 				}
// 				TRW[it][ny][nx] = true
// 			}
// 		}

// 	}
// }
