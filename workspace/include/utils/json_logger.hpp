#ifndef JSON_LOGGER_H_
#define JSON_LOGGER_H_

#include "primary.h"
#include <fstream>
#include <jsoncpp/json/writer.h>
#include <string>

/**
 * Log the performance data into JSON files for visualising
 */
class JsonLogger
{

public:
    JsonLogger()
        : x_data(Json::arrayValue),
          y_data(Json::arrayValue),
          vel_data(Json::arrayValue),
          cte_data(Json::arrayValue),
          etheta_data(Json::arrayValue),
          costs_data(Json::arrayValue)
    {
    }

    void logWeights(double w_vel, double w_cte, double w_etheta, double w_omega, double w_acc, double w_omega_d, double w_acc_d)
    {
        // vel, cte, etheta, omega, acc, omega_d, acc_d
        root["weights"]["vel"] = w_vel;
        root["weights"]["cte"] = w_cte;
        root["weights"]["etheta"] = w_etheta;
        root["weights"]["omega"] = w_omega;
        root["weights"]["acc"] = w_acc;
        root["weights"]["omega_d"] = w_omega_d;
        root["weights"]["acc_d"] = w_acc_d;
    }

    void logX(double x)
    {
        x_data.append(Json::Value(x));
    }

    void logY(double y)
    {
        y_data.append(Json::Value(y));
    }

    void logVelError(double vel)
    {
        vel_data.append(Json::Value(vel));
    }

    void logCte(double cte)
    {
        cte_data.append(Json::Value(cte));
    }

    void logEtheta(double etheta)
    {
        etheta_data.append(Json::Value(etheta));
    }

    void logCost(double cost)
    {
        costs_data.append(Json::Value(cost));
    }

    /**
     * Save to file
     * 
     * @param filepath: Path of the file to save data to
     * 
     * @return True on success, false otherwise
     */
    bool dump(std::string filepath)
    {
        root["x"] = x_data;
        root["y"] = y_data;
        root["vel"] = vel_data;
        root["cte"] = cte_data;
        root["etheta"] = etheta_data;
        root["costs"] = costs_data;

        try
        {
            std::ofstream outFile;
            Json::StyledStreamWriter writer;

            outFile.open(filepath);
            writer.write(outFile, root);
            outFile.close();
        }
        catch (std::exception &e)
        {
            CONSOLE_LOG("[ ERROR ]: Could not save data - " << e.what() << std::endl);
            return false;
        }

        return true;
    }

private:
    Json::Value root;
    Json::Value x_data;
    Json::Value y_data;
    Json::Value vel_data;
    Json::Value cte_data;
    Json::Value etheta_data;
    Json::Value costs_data;
};

#endif
