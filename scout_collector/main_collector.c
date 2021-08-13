#include <stdio.h>
#include <stdlib.h>
#include "main_collector.h"

/***************************MACRO DEFINITION**********************************/
#define MAX_TASKS_SCOUT         5
#define MAX_TASKS_COLLECTOR     4
#define RUN_TIME                50

//#define SCHEDULE_SCOUT
#define SCHEDULE_COLLECTOR

/*****************************************************************************/

/*********************STRUCTURE***********************************************/


/***************************************FUNCTION DECLARATION******************/
/*Scout tasks*/
void TASK_1_SCOUT(void);
void TASK_2_SCOUT(void);
void TASK_3_SCOUT(void);
void TASK_4_SCOUT(void);
void TASK_IDLE_SCOUT(void);

/*collector tasks*/
void TASK_1_COLLECTOR(void);
void TASK_2_COLLECTOR(void);
void TASK_3_COLLECTOR(void);
void TASK_IDLE_COLLECTOR(void);

struct task* accept(struct task* head, int index, struct task*);
void sort(struct task* start);
void swap(struct task* task1, struct task* task2);
void ready_isr(struct task*);

struct task* schedule_scout(struct task*, int);
struct task* schedule_collector(struct task*, int);

/*****************************************************************************/

/*********************************GLOBAL VARIABLES****************************/

int col_running = 0;

/**********************************TASKS**************************************/
static task TASKS_SCOUT[] = { 
        { 1, 1, 4, 1, 1, 1, 0, 1, TASK_1_SCOUT },
        { 2, 0, 4, 0, 1, 0, 0, 1, TASK_2_SCOUT },
        { 3, 2, 4, 1, 1, 1, 0, 1, TASK_3_SCOUT },
        { 4, 3, 4, 1, 1, 1, 0, 1, TASK_4_SCOUT },
        { 5, 4, 4, 0, 1, 0, 0, 1, TASK_IDLE_SCOUT } };

static task TASKS_COLLECTOR[] = {
        { 1, 0, 4, 0, 1, 0, 0, 1, TASK_1_COLLECTOR },
        { 2, 1, 4, 1, 2, 1, 0, 2, TASK_2_COLLECTOR },
        { 3, 2, 4, 1, 1, 1, 0, 1, TASK_3_COLLECTOR },
        { 4, 3, 4, 0, 1, 0, 0, 1, TASK_IDLE_COLLECTOR } };
/*****************************************************************************/

/*****************************MAIN********************************************/
int main(void)
{
    init();
}

int init(void) {
    struct task* head_scout = NULL;


    struct task* head_collector = NULL;


    struct task* schedule_task_scout;
    struct task* schedule_task_collector;

    int count = 0;
    start_scout = NULL;
    start_collector = NULL;

    /*accept and sort scout tasks*/
    while (count < MAX_TASKS_SCOUT) {
        head_scout = accept(head_scout, count, TASKS_SCOUT);
        if (start_scout == NULL) {
            start_scout = head_scout;
        }
        count++;
    }
    sort(start_scout);

    /*accept and sort collector tasks*/
    count = 0;
    while (count < MAX_TASKS_COLLECTOR) {
        head_collector = accept(head_collector, count, TASKS_COLLECTOR);
        if (start_collector == NULL) {
            start_collector = head_collector;
        }
        count++;
    }
    sort(start_collector);
#if 0
    int time = 0;
    while (time < RUN_TIME) {
        printf("\n\ntime:%d", time);
#ifdef SCHEDULE_SCOUT
        schedule_task_scout = schedule_scout(start_scout, time);
        schedule_task_scout->func();
#endif
#ifdef SCHEDULE_COLLECTOR
        schedule_task_collector = schedule_collector(start_collector, time);
        schedule_task_collector->func();
#endif
        time++;
    }
#endif
    return 1;
}

int schedule_collector_wrapper(int time)
{
    struct task* schedule_task_collector = schedule_collector(start_collector, time);
    return schedule_task_collector->id;
}

/*****************************************************************************/

/************************FUNCTION DEFINITION**********************************/

/*SCOUT tasks*/
void TASK_1_SCOUT() {
    printf("\nSCOUT TASK 1 active : Sensor check");
}
void TASK_2_SCOUT() {
    printf("\nSCOUT TASK 2 active : RF communication");
}

void TASK_3_SCOUT() {
    printf("\nSCOUT TASK 3 active : Send result to Collector");
}

void TASK_4_SCOUT() {
    printf("\nSCOUT TASK 4 active : movement");
}

void TASK_IDLE_SCOUT() {
    printf("\nSCOUT TASK IDLE active");
}

/*COLLECTOR TASKS*/
void TASK_1_COLLECTOR() {
    printf("\nCOLLECTOR TASK 1 active : RF Communication");
}
void TASK_2_COLLECTOR() {
    printf("\nCOLLECTOR TASK 2 active : Proximity sensor processing");
}

void TASK_3_COLLECTOR() {
    printf("\nCOLLECTOR TASK 3 active : movement");
}

void TASK_IDLE_COLLECTOR() {
    printf("\nCOLLECTOR TASK IDLE active");
}

/*SCOUT scheduling*/
struct task* schedule_scout(struct task* start, int time) {
    static int running = 0;
    struct task* temp;
    struct task* to_schedule;
    int rand_num;
    rand_num = rand() % 12 + 1;
    printf("\nrand_num for scout : %d", rand_num);
    temp = start;
    while (temp->next != NULL) {
        if ((time % temp->period == 0) && (temp->periodic == 1)) {
            temp->ready = 1;
        } else if ((temp->periodic == 0) && time % rand_num == 0) {
            ready_isr(temp);
        }
        temp = temp->next;
    }

    temp = start;
    if (running == 1) {
        while (temp->next != NULL) {
            if (temp->running == 1) {
                to_schedule = temp;
                temp->cost--;
                if (temp->cost == 0) {
                    temp->cost = temp->restore;
                    running = 0;
                    temp->ready = 0;
                }
                break;
            } else
                temp = temp->next;
        }
    } else {
        while (temp->next != NULL) {
            if (temp->ready == 1) {
                to_schedule = temp;
                temp->cost--;
                temp->running = 1;
                running = 1;
                if (temp->cost == 0) {
                    temp->ready = 0;
                    running = 0;
                    temp->running = 0;
                    temp->cost = temp->restore;
                }
                break;
            } else {
                to_schedule = temp->next; /*Schedule last task continue*/
                temp = temp->next;
            }
        }
    }

    return to_schedule;
}

/*Accept tasks*/
struct task* accept(struct task* head, int index, struct task* TASKS) {
    struct task* new_task = (struct task*) malloc(sizeof(struct task));
    if (head == NULL) {
        head = new_task;
    } else {
        head->next = new_task;
    }

    new_task->id = TASKS[index].id;
    new_task->priority = TASKS[index].priority;
    new_task->period = TASKS[index].period;
    new_task->ready = TASKS[index].ready;
    new_task->periodic = TASKS[index].periodic;
    new_task->func = TASKS[index].func;
    new_task->cost = TASKS[index].cost;
    new_task->running = TASKS[index].running;
    new_task->restore = TASKS[index].restore;
    new_task->next = NULL;
    return new_task;

}
;

/*Sort tasks*/
void sort(struct task* start) {
    struct task* temp_1 = start;
    struct task* temp_2;

    for (temp_1 = start; temp_1 != NULL; temp_1 = temp_1->next) {
        for (temp_2 = temp_1->next; temp_2 != NULL; temp_2 = temp_2->next) {
            if (temp_1->priority > temp_2->priority) {
                swap(temp_1, temp_2);
            }
        }
    }
}

/*Swap*/
void swap(struct task* task1, struct task* task2) {

    int id;
    int priority;
    int period;
    int periodic;
    int ready;
    int cost;
    int running;
    int restore;
    void (*func)(void);

    id = task1->id;
    period = task1->period;
    priority = task1->priority;
    periodic = task1->periodic;
    ready = task1->ready;
    cost = task1->cost;
    func = task1->func;
    running = task1->running;
    restore = task1->restore;

    task1->id = task2->id;
    task1->period = task2->period;
    task1->priority = task2->priority;
    task1->periodic = task2->periodic;
    task1->ready = task2->ready;
    task1->cost = task2->cost;
    task1->func = task2->func;
    task1->running = task2->running;
    task1->restore = task2->restore;

    task2->id = id;
    task2->period = period;
    task2->priority = priority;
    task2->periodic = periodic;
    task2->ready = ready;
    task2->cost = cost;
    task2->func = func;
    task2->running = running;
    task2->restore = restore;

}

/*ISR for aperiodic tasks*/
void ready_isr(struct task* ap_task) {
    ap_task->ready = 1;
}

/*COLLECTOR scheduling*/
struct task* schedule_collector(struct task* start, int time) {
    struct task* temp;
    struct task* to_schedule;
    struct task* store = NULL;

    int rand_num;
    rand_num = rand() % 12 + 1;
    printf("\n\nrand_num for collector : %d", rand_num);
    temp = start;
    while (temp->next != NULL) {
        if ((time % temp->period == 0) && (temp->periodic == 1)) {
            temp->ready = 1;
        } else if ((temp->periodic == 0) && time % rand_num == 0) {
            ready_isr(temp);
        }
        temp = temp->next;
    }
    temp = start;

    /*if some task is already running, store that task*/
    if (col_running == 1) {
        while (temp->next != NULL) {

            if (temp->running == 1) {
                store = temp;
                break;
            } else {
                store = temp;

                temp = temp->next;
            }
        }
    }
    temp = start;
    while (temp->next != NULL) {
        if (temp->ready == 1) {
            to_schedule = temp;
            temp->cost--;
            temp->running = 1;
            col_running = 1;

            if (store != NULL) {
                if (temp->id != store->id && temp->priority < store->priority) {
                    printf("\t\t\t\tpreempted ongoing task");
                }
            }
            if (temp->cost == 0) {
                temp->running = 0;
                temp->ready = 0;
                temp->cost = temp->restore;
                col_running = 0;
            }
            break;
        }

        else {
            to_schedule = temp->next;
            temp = temp->next;
        }
    }
    return to_schedule;
}

/*****************************************************************************/
