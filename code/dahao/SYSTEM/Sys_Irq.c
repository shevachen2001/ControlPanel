
#include "SYSTEM\sys.h"

/**
  * @brief Auto Wake Up Interruption routine.
  * @par Parameters:
  * None
  * @retval
  * None
  */
#ifdef _COSMIC_
@far @interrupt void AWU_IRQHandler(void)
#endif

#ifdef _RAISONANCE_
void AWU_IRQHandler(void) interrupt 1
#endif	

#ifdef _IAR_SYSTEMS_
#pragma vector=3
__interrupt void AWU_IRQHandler(void)
#endif
{
	AWU_GetFlagStatus();
}

