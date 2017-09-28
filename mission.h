#ifndef MISSION_H
#define	MISSION_H

#include "localmap.h"
#include "config.h"
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "dlite.h"
#include "xmllogger.h"

class Mission
{
    public:
        Mission();
        Mission (const char* fileName);
        ~Mission();

        bool getMap();
        bool getConfig();
        bool getLocalMap();
        bool createLog();
        void createSearch();
        void createEnvironmentOptions();
        void startSearch();
        void printSearchResultsToConsole();
        void saveSearchResultsToLog();

    private:

        Map                     map;
        LocalMap                localmap;
        Config                  config;
        EnvironmentOptions      options;
        Dlite                   dlitesearch;
        ILogger*                logger;
        const char*             fileName;
        SearchResult            sr;
};

#endif

