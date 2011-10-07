/*
 * SensorReadingQueue.h
 *
 *  Created on: Aug 8, 2010
 *      Author: kruset
 */

#ifndef ARRAYQ_H_
#define ARRAYQ_H_

// -100 % 3 can be -1 or 2, depending on OS, we make sure we always get 2
#define MODULO_POSITIVE(x, n)       (((x) >= 0.0) ? ((x) % (n)) : ((((x) % (n)) + (n)) % (n)))
/*
 * SensorReadingQueue.h
 *
 * This is a cyclic buffer implementation for sensor readings.
 *
 * It allows pushing of new entries, which drops oldest if maxsize is reached, explicit consuming is not possible
 *
 * The n last pushes may not be validated yet,they can be validated or purged.
 * This is to allow recording values that might be the result of noise, and
 * defer the decision about whether to drop or not based on future readings.
 */
template<typename T, int N> class SensorReadingQueue {
  T _base[N];
  int _front;
  int _back;
  int _size;

  int _frontValid;
  int _sizeValid;

public:
  SensorReadingQueue()
  :
    _front(0), _back(0), _size(0), _frontValid(0), _sizeValid(0)
  {
  }

  bool empty() const
  {
    return _size == 0;
  }

  bool full() const
  {
    return _size == N;
  }

  int size() const
  {
    return _size;
  }

  int sizeValid() const
  {
    return _sizeValid;
  }

  /**
   * all pushed entries not yet valid are validated
   */
  void validateAll()
  {
    _frontValid = _front;
    _sizeValid=_size;
  }

  /**
   * removes entries not yet valid
   */
  void purgeNonValidated()
  {
    _front = _frontValid;
    _size = _sizeValid;
  }

  /**
   * removes entries that had been validated
   */
  void purgeValids()
  {
    _back = _frontValid;
    _size = _size - _sizeValid;
    _sizeValid = 0;// no valids left
  }


  void push(T data)
  {
    if (full()) {
      // make room by dropping last
      if (++_back == N) {
           _back = 0;
      }
      // dropping means one valid less,if there is any
      if (_sizeValid > 0) {
        _sizeValid--;
      }
    } else {
      ++_size;
    }
    _base[_front] = data;
    if (++_front == N)
      _front = 0;

  }


  /**
   * returns the nth element from back, meaning if we push a, b, c, elementAt(0) => a;
   * always returns an element from the backing array, might return invalid values though.
   */
  T elementAt(int i)
  {
    int index = MODULO_POSITIVE(_back + i, N);
    return _base[index];
  }

  /**
   * returns the nth element from front, meaning if we push a, b, c, elementAt(0) => c; elementAt(1) => b;
   * always returns an element from the backing array, might return invalid values though.
   */
  T elementAtR(int i)
  {
    int index = MODULO_POSITIVE((_front - 1) - i, N);
    return _base[index];
  }

  /**
   * returns the nth valid element from front, meaning if we push a, b, c, d, (d not validated), validElementAtR(0) => c;validElementAtR(1) => b;
   * always returns an element from the backing array, might return invalid values though.
   */
  T validElementAtR(int i)
  {
    int index = MODULO_POSITIVE((_frontValid- 1) - i, N);
    return _base[index];
  }


};




#endif /* ARRAYQ_H_ */
