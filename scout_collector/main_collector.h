typedef struct task {
    int id;
    int priority;
    int period;
    int ready;
    int cost;
    int periodic;
    int running;
    int restore;
    void (*func)(void);
    struct task *next;
} task;

struct task* start_scout;
struct task* start_collector;

void sort(struct task* start);

struct task* accept(struct task* head, int index, struct task* TASKS);

struct task* schedule_collector(struct task* start, int time);