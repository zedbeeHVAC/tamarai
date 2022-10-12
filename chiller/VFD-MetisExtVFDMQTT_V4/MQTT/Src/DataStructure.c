#include "DataStructure.h"
#include "main.h"

static struct BoardConfig st_boardparams;

void
fvBoardParamsRead(struct BoardConfig *BoardConfig_data)
{
  memcpy(BoardConfig_data,&st_boardparams,sizeof(struct BoardConfig));
}

void
fvBoardParamsWrite(struct BoardConfig *BoardConfig_data)
{
   memcpy(&st_boardparams,BoardConfig_data,sizeof(struct BoardConfig));
}