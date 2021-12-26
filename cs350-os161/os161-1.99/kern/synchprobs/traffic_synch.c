#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>
#include <queue.h>
/*
   north = 0,
   east = 1,
   south = 2,
   west = 3
 */
struct cv *N;
struct cv *S;
struct cv *E;
struct cv *W;
//conditional variable for north, south, east, west
//intersection state keeps track of what cars(from where) are currently in the intersection
static int int_state[4];
//line indicates 4 the # of cars lining up in 4 directions
static int line[4];	//
//traffic light shows what direction is the current traffic flow.
static int light;
const int empty_state = -1;
static struct lock *int_lk;

/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 */
void intersection_sync_init(void) {
	int_lk = lock_create("Intersection Lock");
	if (int_lk == NULL) {
		panic("int_lk can't be created");
	}

	S = cv_create("South");
	if (S == NULL) {
		panic("South_cv can't be created");
	}
	N = cv_create("North");
	if (N == NULL) {
		panic("Nouth_cv can't be created");
	}
	E = cv_create("East");
	if (E == NULL) {
		panic("East_cv can't be created");
	}

	W = cv_create("West");
	if (W == NULL) {
		panic("West_cv can't be created");
	}
	light = empty_state; //empty state
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void intersection_sync_cleanup(void) {
	KASSERT(int_lk != NULL);
	KASSERT(S != NULL);
	KASSERT(N != NULL);
	KASSERT(E != NULL);
	KASSERT(W != NULL);
	lock_destroy(int_lk);
	cv_destroy(S);
	cv_destroy(E);
	cv_destroy(W);
	cv_destroy(N);
}


/*
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void intersection_before_entry(Direction origin, Direction destination) {
	(void) destination;
	KASSERT(int_lk != NULL);
	KASSERT(S != NULL);
	KASSERT(N != NULL);
	KASSERT(E != NULL);
	KASSERT(W != NULL);
	lock_acquire(int_lk);
	if (origin == north) {
		if((light != north && light != empty_state) || int_state[east] != 0 || int_state[west] != 0 || int_state[south] != 0) {
			line[0]++;
			cv_wait(N, int_lk);
		}
		if (light == empty_state) {
			light = north;
		}
		line[north] = 0;
		int_state[north]++;
	} else if (origin == east) {
		if((light != east && light != empty_state) || int_state[north] != 0 || int_state[south] != 0 || int_state[west] != 0) {
			line[east]++;
			cv_wait(E, int_lk);
		}
		if (light == empty_state) {
			light = east;
		}
		line[east] = 0;
		int_state[east]++;
	} else if (origin == south) {
		if((light != south && light != empty_state) || int_state[north] != 0 || int_state[west] != 0 || int_state[east] != 0) {
			line[south]++;
			cv_wait(S, int_lk);
		}
		if (light == empty_state) {
			light = south;
		}
		line[south] = 0;
		int_state[south]++;
	} else {
		if((light != west && light != empty_state) || int_state[north] != 0 || int_state[east] != 0 || int_state[south] != 0) {
			line[west]++;
			cv_wait(W, int_lk);
		}
		if (light == empty_state) {
			light = west;
		}
		line[west] = 0;
		int_state[west]++;
	}
	lock_release(int_lk);
}


/*
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void intersection_after_exit(Direction origin, Direction destination) {
	KASSERT(int_lk != NULL);
	KASSERT(S != NULL);
	KASSERT(N != NULL);
	KASSERT(E != NULL);
	KASSERT(W != NULL);
	(void)destination; 
	lock_acquire(int_lk);
	//a car from north exited
	if (origin == north) {
		int_state[north]--;
	}
	//a car from south exited
	else if (origin == south) {
		int_state[south]--;
	}
	else if (origin == east) {
		int_state[east]--;
	}
	else {
		int_state[west]--;
	}
	if (int_state[north] == 0 && int_state[south] == 0 && int_state[west] == 0 && int_state[east] == 0) {
		if (line[north] == 0 && line[east] == 0 && line[west] == 0 && line[south] == 0) {
			light = empty_state;
			lock_release(int_lk);// no cars in the intersection; no cars waiting
			return;
		}
		switch (origin) {
			case north:	//a car from north exits
				if (line[south] > 0) {
					light = south;
					cv_broadcast(S, int_lk);
				}
				else if (line[east] > 0) {
					light = east;
					cv_broadcast(E, int_lk);
				}
				else {
					light = west;
					cv_broadcast(W, int_lk);
				}
				break;
			case south:
				if (line[east] > 0) {
					light = east;
					cv_broadcast(E, int_lk);
				}
				else if (line[west] > 0) {
					light = west;
					cv_broadcast(W, int_lk);
				}
				else {
					light = north;
					cv_broadcast(N, int_lk);
				}
				break;
			case west:
				if (line[north] > 0) {
					light = north;
					cv_broadcast(N, int_lk);
				}
				else if (line[south] > 0) {
					light = south;
					cv_broadcast(S, int_lk);
				}
				else {
					light = east;
					cv_broadcast(E, int_lk);
				}
				break;
			case east:
				if (line[west] > 0) {
					light = west;
					cv_broadcast(W, int_lk);
				}
				else if (line[north] > 0) {
					light = north;
					cv_broadcast(N, int_lk);
				}
				else {
					light = south;
					cv_broadcast(S, int_lk);
				}
				break;
		}
	}
	lock_release(int_lk);

}
