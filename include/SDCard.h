#ifndef SDCARD_H_INCLUDED
#define SDCARD_H_INCLUDED

#include "sam.h"
#include "Driver_MCI.h"

int32_t SDC_Initialize(ARM_DRIVER_MCI *mci);

void MCI_event (uint32_t event);

#endif /* SDCARD_H_INCLUDED */
