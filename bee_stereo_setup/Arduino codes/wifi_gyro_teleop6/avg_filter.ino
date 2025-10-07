//filter parameters
struct MovingAverageFilter {
  float buffer[10];
  int index;
  int count;

  MovingAverageFilter() : index(0), count(0) {
    for (int i = 0; i < 10; i++) buffer[i] = 0;
  }

  float update(float value) {
    buffer[index] = value;
    index = (index + 1) % 10;
    if (count < 10) count++;

    float sum = 0;
    for (int i = 0; i < count; i++) {
      sum += buffer[i];
    }
    return sum / count;
  }
};