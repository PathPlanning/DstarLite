#include "environmentoptions.h"

EnvironmentOptions::EnvironmentOptions()
{
    metrictype = CN_SP_MT_EUCL;
    allowsqueeze = false;
    allowdiagonal = true;
    cutcorners = false;
    allowjump = true;
}

EnvironmentOptions::EnvironmentOptions(bool AS, bool AD, bool CC, int MT, bool AJ)
{
    metrictype = MT;
    allowsqueeze = AS;
    allowdiagonal = AD;
    cutcorners = CC;
    allowjump = AJ;
}

