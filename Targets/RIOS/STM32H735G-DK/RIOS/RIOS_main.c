

#include "stm32h7xx.h"

#define ENABLE_INTERRUPTS __asm volatile("cpsie i" : : : "memory");
#define DISABLE_INTERRUPTS __asm volatile("cpsid i" : : : "memory");

typedef struct task {
  unsigned long period;      // Rate at which the task should tick
  unsigned long elapsedTime; // Time since task's last tick
  void (*TickFct)(void);     // Function to call for task's tick
} task;

const unsigned char tasksNum = 2;

task tasks[tasksNum];
const unsigned long tasksPeriodGCD = 200; // Timer tick rate
const unsigned long periodToggle = 1000;
const unsigned long periodSequence = 200;

void TickFct_Toggle(void);
void TickFct_Sequence(void);

unsigned char processingRdyTasks = 0;

void Start_RTOS() {
  // Priority assigned to lower position tasks in array
  unsigned char i = 0;
  tasks[i].period = periodSequence;
  tasks[i].elapsedTime = tasks[i].period;
  tasks[i].TickFct = &TickFct_Sequence;
  ++i;
  tasks[i].period = periodToggle;
  tasks[i].elapsedTime = tasks[i].period;
  tasks[i].TickFct = &TickFct_Toggle;
  TimerSet(tasksPeriodGCD);
  TimerOn();
  while (1) {
    Sleep();
  }
}






void TimerISR() {
  unsigned char i;
  if (processingRdyTasks) {
    printf("Timer ticked before task processing done.\n");
  } else {
    // Heart of the scheduler code
    processingRdyTasks = 1;
    for (i = 0; i < tasksNum; ++i) {
      if (tasks[i].elapsedTime >= tasks[i].period) {
        // Ready
        tasks[i].TickFct(); // execute task tick
        tasks[i].elapsedTime = 0;
      }
      tasks[i].elapsedTime += tasksPeriodGCD;
    }
    processingRdyTasks = 0;
  }
}
