#ifndef SEND_H
#define SEND_H

void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size);
void serial_start(void);

#endif /* SEND_H */
