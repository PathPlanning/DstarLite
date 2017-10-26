#include "config.h"
#include "gl_const.h"
#include "tinyxml2.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <math.h>

Config::Config()
{
    LogParams = nullptr;
    SearchParams = nullptr;
}

Config::~Config()
{
    if (SearchParams) delete[] SearchParams;
    if (LogParams) delete[] LogParams;
}

bool Config::getConfig(const char *FileName)
{
    std::string value;
    std::stringstream stream;
    tinyxml2::XMLElement *root = 0, *algorithm = 0, *element = 0, *options = 0;

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS) {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }

    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root) {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' element found in XML file!" << std::endl;
        return false;
    }

    algorithm = root->FirstChildElement(CNS_TAG_ALG);
    if (!algorithm) {
        std::cout << "Error! No '" << CNS_TAG_ALG << "' tag found in XML file!" << std::endl;
        return false;
    }

    N = 8;
    SearchParams = new double[N];

    element = algorithm->FirstChildElement(CNS_TAG_RA);
    if (!element) {
        std::cout << "Warning! No '" << CNS_TAG_RA << "' tag found in algorithm section." << std::endl;
        std::cout << "Value of '" << CNS_TAG_RA << "' was defined to 5." << std::endl;
        SearchParams[CN_SP_RA] = 5;
    }
    else {
        stream << element->GetText();
        stream >> SearchParams[CN_SP_RA];
        stream.str("");
        stream.clear();

        if (SearchParams[CN_SP_RA] < 1) {
            std::cout << "Warning! Value of '" << CNS_TAG_RA << "' tag is not correctly specified. Should be >= 1."
                          << std::endl;
            std::cout << "Value of '" << CNS_TAG_RA << "' was defined to 1." << std::endl;
            SearchParams[CN_SP_RA] = 1;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_HW);
    if (!element) {
        std::cout << "Warning! No '" << CNS_TAG_HW << "' tag found in algorithm section." << std::endl;
        std::cout << "Value of '" << CNS_TAG_HW << "' was defined to 1." << std::endl;
        SearchParams[CN_SP_HW] = 1;
    }
    else {
        stream << element->GetText();
        stream >> SearchParams[CN_SP_HW];
        stream.str("");
        stream.clear();

        if (SearchParams[CN_SP_HW] < 1) {
            std::cout << "Warning! Value of '" << CNS_TAG_HW << "' tag is not correctly specified. Should be >= 1."
                          << std::endl;
            std::cout << "Value of '" << CNS_TAG_HW << "' was defined to 1." << std::endl;
            SearchParams[CN_SP_HW] = 1;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_MT);
    if (!element) {
        std::cout << "Warning! No '" << CNS_TAG_MT << "' tag found in XML file." << std::endl;
        std::cout << "Value of '" << CNS_TAG_MT << "' was defined to 'euclidean'." << std::endl;
        SearchParams[CN_SP_MT] = CN_SP_MT_EUCL;
     } else {
            if (element->GetText())
                value = element->GetText();
            std::transform(value.begin(), value.end(), value.begin(), ::tolower);
            if (value == CNS_SP_MT_MANH) SearchParams[CN_SP_MT] = CN_SP_MT_MANH;
            else if (value == CNS_SP_MT_EUCL) SearchParams[CN_SP_MT] = CN_SP_MT_EUCL;
            else if (value == CNS_SP_MT_DIAG) SearchParams[CN_SP_MT] = CN_SP_MT_DIAG;
            else if (value == CNS_SP_MT_CHEB) SearchParams[CN_SP_MT] = CN_SP_MT_CHEB;
            else {
                std::cout << "Warning! Value of'" << CNS_TAG_MT << "' is not correctly specified." << std::endl;
                std::cout << "Value of '" << CNS_TAG_MT << "' was defined to 'euclidean'" << std::endl;
                SearchParams[CN_SP_MT] = CN_SP_MT_EUCL;
            }
            if (SearchParams[CN_SP_ST] == CN_SP_ST_TH && SearchParams[CN_SP_MT] != CN_SP_MT_EUCL) {
                std::cout << "Warning! This type of metric is not admissible for Theta*!" << std::endl;
            }
    }

    element = algorithm->FirstChildElement(CNS_TAG_JU);
    if (!element) {
        std::cout << "Warning! No '" << CNS_TAG_JU << "' element found in XML file." << std::endl;
        std::cout << "Value of '" << CNS_TAG_JU << "' was defined to default - true" << std::endl;
        SearchParams[CN_SP_JU] = 1;
    }
    else {
        std::string check;
        stream << element->GetText();
        stream >> check;
        stream.clear();
        stream.str("");

        if (check != "1" && check != "true" && check != "0" && check != "false") {
            std::cout << "Warning! Value of '" << CNS_TAG_JU << "' is not correctly specified." << std::endl;
            std::cout << "Value of '" << CNS_TAG_JU << "' was defined to default - false " << std::endl;
            SearchParams[CN_SP_JU] = 0;
        }
        else if (check == "1" || check == "true")
            SearchParams[CN_SP_JU] = 1;
        else
            SearchParams[CN_SP_JU] = 0;
    }


    element = algorithm->FirstChildElement(CNS_TAG_AD);
    if (!element) {
        std::cout << "Warning! No '" << CNS_TAG_AD << "' element found in XML file." << std::endl;
        std::cout << "Value of '" << CNS_TAG_AD << "' was defined to default - true" << std::endl;
        SearchParams[CN_SP_AD] = 1;
    }
    else {
        std::string check;
        stream << element->GetText();
        stream >> check;
        stream.clear();
        stream.str("");

        if (check != "1" && check != "true" && check != "0" && check != "false") {
            std::cout << "Warning! Value of '" << CNS_TAG_AD << "' is not correctly specified." << std::endl;
            std::cout << "Value of '" << CNS_TAG_AD << "' was defined to default - true " << std::endl;
            SearchParams[CN_SP_AD] = 1;
        }
        else if (check == "1" || check == "true")
            SearchParams[CN_SP_AD] = 1;
        else
            SearchParams[CN_SP_AD] = 0;
    }

    if (SearchParams[CN_SP_AD] == 0) {
        SearchParams[CN_SP_CC] = 0;
        SearchParams[CN_SP_AS] = 0;
    }
    else {
        element = algorithm->FirstChildElement(CNS_TAG_CC);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_CC << "' element found in XML file." << std::endl;
            std::cout << "Value of '" << CNS_TAG_CC << "' was defined to default - false" << std::endl;
            SearchParams[CN_SP_CC] = 0;
        }
        else {
            std::string check;
            stream << element->GetText();
            stream >> check;
            stream.clear();
            stream.str("");
            if (check != "1" && check != "true" && check != "0" && check != "false") {
                std::cout << "Warning! Value of '" << CNS_TAG_CC << "' is not correctly specified." << std::endl;
                std::cout << "Value of '" << CNS_TAG_CC << "' was defined to default - false" << std::endl;
                SearchParams[CN_SP_CC] = 0;
            }
            else {
                if (check == "1" || check == "true")
                    SearchParams[CN_SP_CC] = 1;
                else
                    SearchParams[CN_SP_CC] = 0;
            }
        }
        if (SearchParams[CN_SP_CC] == 0) {
            SearchParams[CN_SP_AS] = 0;
        }
        else {
            element = algorithm->FirstChildElement(CNS_TAG_AS);
            if (!element) {
                std::cout << "Warning! No '" << CNS_TAG_AS << "' element found in XML file." << std::endl;
                std::cout << "Value of '" << CNS_TAG_AS << "' was defined to default - false." << std::endl;
                SearchParams[CN_SP_AS] = 0;
            }
            else {
                std::string check;
                stream << element->GetText();
                stream >> check;
                stream.clear();
                stream.str("");
                if (check != "1" && check != "true" && check != "0" && check != "false") {
                    std::cout << "Warning! Value of '" << CNS_TAG_AS << "' is not correctly specified." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_AS << "' was defined to default - false." << std::endl;
                    SearchParams[CN_SP_AS] = 0;
                }
                else {
                    if (check == "1" || check == "true")
                        SearchParams[CN_SP_AS] = 1;
                    else
                        SearchParams[CN_SP_AS] = 0;
                }
            }
        }
    }

    options = root->FirstChildElement(CNS_TAG_OPT);
    LogParams = new std::string[3];
    LogParams[CN_LP_PATH] = "";
    LogParams[CN_LP_NAME] = "";

    if (!options) {
        std::cout << "Warning! No '" << CNS_TAG_OPT << "' tag found in XML file." << std::endl;
        std::cout << "Value of '" << CNS_TAG_LOGLVL << "' tag was defined to 'short log' (1)." << std::endl;
        LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_SHORT_WORD;
    }
    else {
        element = options->FirstChildElement(CNS_TAG_LOGLVL);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_LOGLVL << "' tag found in XML file." << std::endl;
            std::cout << "Value of '" << CNS_TAG_LOGLVL << "' tag was defined to 'short log' (1)." << std::endl;
            LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_SHORT_WORD;
        }
        else {
            stream << element->GetText();
            stream >> value;
            stream.str("");
            stream.clear();
            //std::transform(value.begin(), value.end(), value.begin(), ::tolower);
            if (value == CN_LP_LEVEL_NOPE_WORD || value == CN_LP_LEVEL_NOPE_VALUE)
                LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_NOPE_WORD;
            else if (value == CN_LP_LEVEL_TINY_WORD || value == CN_LP_LEVEL_TINY_VALUE)
                LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_TINY_WORD;
            else if (value == CN_LP_LEVEL_SHORT_WORD || value == CN_LP_LEVEL_SHORT_VALUE)
                LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_SHORT_WORD;
            else if (value == CN_LP_LEVEL_MEDIUM_WORD || value == CN_LP_LEVEL_MEDIUM_VALUE)
                LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_MEDIUM_WORD;
            else if (value == CN_LP_LEVEL_FULL_WORD || value == CN_LP_LEVEL_FULL_VALUE)
                LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_FULL_WORD;
            else {
                std::cout << "'" << CNS_TAG_LOGLVL << "' is not correctly specified" << std::endl;
                std::cout << "Value of '" << CNS_TAG_LOGLVL << "' tag was defined to 'short log' (1)." << std::endl;
                LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_SHORT_WORD;
            }
            std::cout << LogParams[CN_LP_LEVEL] << std::endl;
        }


        element = options->FirstChildElement(CNS_TAG_LOGPATH);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_LOGPATH << "' tag found in XML file." << std::endl;
            std::cout << "Value of '" << CNS_TAG_LOGPATH << "' tag was defined to 'current directory'." << std::endl;
        }
        else if (!element->GetText()) {
            std::cout << "Warning! Value of '" << CNS_TAG_LOGPATH << "' tag is missing!" << std::endl;
            std::cout << "Value of '" << CNS_TAG_LOGPATH << "' tag was defined to 'current directory'." << std::endl;
        }
        else {
            LogParams[CN_LP_PATH] = element->GetText();
        }


        element = options->FirstChildElement(CNS_TAG_LOGFN);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_LOGFN << "' tag found in XML file!" << std::endl;
            std::cout << "Value of '" << CNS_TAG_LOGFN
                      << "' tag was defined to default (original filename +'_log' + original file extension."
                      << std::endl;
        }
        else if (!element->GetText()) {
            std::cout << "Warning! Value of '" << CNS_TAG_LOGFN << "' tag is missing." << std::endl;
            std::cout << "Value of '" << CNS_TAG_LOGFN
                      << "' tag was defined to default (original filename +'_log' + original file extension."
                      << std::endl;
        }
        else
            LogParams[CN_LP_NAME] = element->GetText();
    }
    return true;
}
