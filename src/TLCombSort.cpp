//
//    FILE: sort.ino
//  AUTHOR: Rob Tillaart
// VERSION: 0.1.00
// PURPOSE: compare sorting methods
//    DATE: 2014-11-20
//     URL: http://forum.arduino.cc/index.php?topic=280486.0
//
// Released to the public domain
//

#include "TLCombSort.h"

void combSort(int32_t *ar, uint8_t n)
{
  uint8_t i, j, gap, swapped = 1;
  int32_t temp;

  gap = n;
  while (gap > 1 || swapped == 1)
  {
    gap = gap * 10 / 13;
    if (gap == 9 || gap == 10) gap = 11;
    if (gap < 1) gap = 1;
    swapped = 0;
    for (i = 0, j = gap; j < n; i++, j++)
    {
      if (ar[i] > ar[j])
      {
        temp = ar[i];
        ar[i] = ar[j];
        ar[j] = temp;
        swapped = 1;
      }
    }
  }
}


