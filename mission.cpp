#include "mission.h"
#include "xmllogger.h"
#include "gl_const.h"

Mission::Mission()
{
    logger = nullptr;
    fileName = nullptr;
    correct = false;
}

Mission::Mission(const char *FileName, double rad)
{
    fileName = FileName;
    logger = nullptr;
    correct = false;
    radius = rad;
}

Mission::~Mission()
{
    if (logger)
        delete logger;
}

bool Mission::getMap()
{
    return map.GetMap(fileName) && localmap.GetLocalMap(map, map.start, radius);
}

bool Mission::getConfig()
{
    return config.getConfig(fileName);
}

bool Mission::createLog()
{
    if (logger != NULL) delete logger;
    logger = new XmlLogger(config.LogParams[CN_LP_LEVEL]);
    return logger->getLog(fileName, config.LogParams);
}

void Mission::createEnvironmentOptions()
{
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_BFS || config.SearchParams[CN_SP_ST] == CN_SP_ST_DIJK) {

        options = EnvironmentOptions(config.SearchParams[CN_SP_AS], config.SearchParams[CN_SP_AD],
                                     config.SearchParams[CN_SP_CC]);

    }

    else
        options = EnvironmentOptions(config.SearchParams[CN_SP_AS], config.SearchParams[CN_SP_AD],
                                     config.SearchParams[CN_SP_CC], config.SearchParams[CN_SP_MT]);
    map.algorithm_info = options;
}

void Mission::createSearch()
{
    dlitesearch = Dlite(radius, config.SearchParams[CN_SP_HW]);
}

void Mission::startSearch()
{
    sr = dlitesearch.FindThePath(localmap, map, options);
}

void Mission::printSearchResultsToConsole()
{
    std::cout << "Path ";
    if (!sr.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    std::cout << "numberofsteps=" << sr.numberofsteps << std::endl;
    std::cout << "nodescreated=" << sr.nodescreated << std::endl;
    if (sr.pathfound) {
        std::cout << "pathlength=" << sr.pathlength << std::endl;
        std::cout << "pathlength_scaled=" << sr.pathlength * map.CellSize << std::endl;
    }
    std::cout << "time=" << sr.time << std::endl;
}

void Mission::saveSearchResultsToLog()
{
    logger->writeToLogSummary(sr.numberofsteps, sr.nodescreated, sr.pathlength, sr.time, map.CellSize);
    if (sr.pathfound) {
        logger->writeToLogPath(*sr.lppath);
        logger->writeToLogHPpath(*sr.hppath);
        logger->writeToLogMap(map, *sr.lppath, true);
    } else {
        logger->writeToLogMap(map, *sr.lppath, false);
        logger->writeToLogNotFound();
    }
    logger->saveLog();
}

const char *Mission::getAlgorithmName()
{
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR)
        return CNS_SP_ST_ASTAR;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_DIJK)
        return CNS_SP_ST_DIJK;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_BFS)
        return CNS_SP_ST_BFS;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_JP_SEARCH)
        return CNS_SP_ST_JP_SEARCH;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_TH)
        return CNS_SP_ST_TH;
    else
        return "";
}
