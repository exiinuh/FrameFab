#include "LinearP.h"

std::ostream& operator<<(std::ostream& out, const LinearP& lp)
{
	out << lp.report();
	return out;
}