#include <stm32f407xx.h>
#include "cmsis_os2.h"  
#include "main.h"  
int main(void)
{

#ifdef RTE_CMSIS_RTOS2
  /* Initialize CMSIS-RTOS2 */
  osKernelInitialize ();

  /* Create application main thread */
  osThreadNew(app_main, NULL, NULL);

  /* Start thread execution */
  osKernelStart();
#endif

  while (1)
  {
  }
}