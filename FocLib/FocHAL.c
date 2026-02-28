#include "FocHAL.h"


/***
 * @brief:FOC硬件抽象层，专注于应用层开发，调用底层FocCore.h文件的底层文件
*/

#define Gain 20.0f
#define Shunt   0.002
#define Factor (Gain*Shunt)


/*电流标幺化*/
