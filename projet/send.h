#ifndef SEND_DATA_H
#define SEND_DATA_H

void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size);
void serial_start(void);

#endif /* SEND_DATA_H */