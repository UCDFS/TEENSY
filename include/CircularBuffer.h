#pragma once
#include "config.h"

template <typename T, size_t size>
class CircularBuffer {
private:
  T _buffer[size];
  size_t _head = 0;
  size_t _tail = 0;
  bool _full = false;
  uint32_t _droppedCount = 0;  // entries silently overwritten because the buffer was full

public:
  void push(T item) {
    _buffer[_head] = item;
    if (_full) {
      _tail = (_tail + 1) % size;
      _droppedCount++;
    }
    _head = (_head + 1) % size;
    _full = _head == _tail;
  }

  bool pop (T &item) {
    if (isEmpty()) {
      return false;
    }

    item = _buffer[_tail];
    _full = false;
    _tail = (_tail + 1) % size;
    return true;
  }

  bool isEmpty() const { return (!_full && (_head == _tail)); }
  bool isFull() const { return _full; }

  // Returns entries dropped since the last call and resets the counter -
  // callers should surface a nonzero result rather than let overflow stay silent.
  uint32_t takeDroppedCount() {
    uint32_t n = _droppedCount;
    _droppedCount = 0;
    return n;
  }
};