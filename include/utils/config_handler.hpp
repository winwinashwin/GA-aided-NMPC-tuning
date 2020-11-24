#ifndef MPC_CONFIG_PARSER_H_
#define MPC_CONFIG_PARSER_H_

#include "pcheaders.h"

/**
 * Runtime configurations
 */
namespace config
{
    /**
     * Type of configuration of interest
     */
    enum ConfigType
    {
        /// Hone the weights using Genetic Algorithm
        GA,
        /// Run and visualise MPC for a given set of params
        MONO
    };

    /**
     * Struct for storing data from the configuration file for genetic algorithm
     */
    struct GeneticAlgorithm
    {
        struct General
        {
            size_t generations, population_size, mating_pool_size, iterations_per_genome;
            bool interactive_decision_tree;
        } general;

        struct Operators
        {
            double mutation_probability, crossover_bias;
        } operators;
    };

    /**
     * Struct for storing data from configuration file for genetic algorithm (MPC configurations)
     */
    struct MPC_Controller_GA
    {
        struct __General
        {
            size_t timesteps;
            double sample_time;

        } general;

        struct __Initial
        {
            double x, y, theta, linear_velocity, angular_velocity, throttle;
        } initial_state;

        struct __Desired
        {
            double velocity, cross_track_error, orientation_error;
        } desired;

        struct __MaxBounds
        {
            double omega, throttle;
        } max_bounds;

        struct __WeightBounds
        {
            std::pair<double, double> w_cte, w_etheta, w_vel, w_omega, w_acc, w_omega_d, w_acc_d;
        } weight_bounds;
    };

    /**
     * Struct for storing data from configuration file for singular run (MPC configurations) (target: mpc_mono)
     */
    struct MPC_Controller_MONO
    {
        struct __General
        {
            size_t timesteps;
            double sample_time;

        } general;

        struct __Initial
        {
            double x, y, theta, linear_velocity, angular_velocity, throttle;
        } initial_state;

        struct __Desired
        {
            double velocity, cross_track_error, orientation_error;
        } desired;

        struct __MaxBounds
        {
            double omega, throttle;
        } max_bounds;

        struct __Weights
        {
            double w_cte, w_etheta, w_vel, w_omega, w_acc, w_omega_d, w_acc_d;
        } weights;

        struct __TeardownTolerance
        {
            double velocity, cross_track_error, orientation_error;
        } teardown_tolerance;
    };

    /**
     * Singleton for handling the configuration as obtained from the YAML config files
     * 
     * @tparam ctype: The mode of operation, either GA or MONO
     */
    template <ConfigType ctype>
    class ConfigHandler
    {

    public:
        /// Delete the copy constructor
        ConfigHandler(const ConfigHandler &) = delete;

        /**
         * Get root node of the tree representing the YAML file parameters
         * 
         * @return Node representing the root
         */
        static YAML::Node Root()
        {
            return get().RootImpl();
        }

        /**
         * Get configuration for Genetic Algorithm
         * 
         * Note the use of std::enable_if. So if the template parameter is config::MONO, this method will not exist and can't be accessed
         * 
         * @return Reference to struct representing the configuration
         */
        template <ConfigType U = ctype>
        typename std::enable_if<U == 0, GeneticAlgorithm &>::type static getGAConfig()
        {
            return get().getGAConfigImpl();
        }

        /**
         * Get MPC configuration for Genetic Algorithm
         * 
         * @return Reference to struct representing the configuration
         */
        template <ConfigType U = ctype>
        typename std::enable_if<U == 0, MPC_Controller_GA &>::type static getMpcConfig()
        {
            return get().getMpcConfigImpl();
        }

        /**
         * Get MPC configuration for singular run
         * 
         * @return Reference to struct representing the configuration
         */
        template <ConfigType U = ctype>
        typename std::enable_if<U == 1, MPC_Controller_MONO &>::type static getMpcConfig()
        {
            return get().getMpcConfigImpl();
        }

    private:
        /// Name of file to parse
        std::string m_filename;
        YAML::Node m_root;
        MPC_Controller_GA m_mpcConfigGA;
        MPC_Controller_MONO m_mpcConfigMono;
        GeneticAlgorithm m_genConfig;

        /// Constructor
        ConfigHandler()
        {
            CONSOLE_LOG("**************************************************************\n");
            CONSOLE_LOG("                     GA AIDED NMPC TUNING                     \n");
            CONSOLE_LOG("                                                              \n");
            CONSOLE_LOG(" Robust gain tuning of Nonlinear MPC using Genetic Algorithms \n");
            CONSOLE_LOG("                                                              \n");
            CONSOLE_LOG("                                    @author: ashwin5059198    \n");
            CONSOLE_LOG("                                                              \n");
            CONSOLE_LOG("**************************************************************\n");

            switch (ctype)
            {
            case GA:
                m_filename = "config/config-ga.yaml";
                break;

            case MONO:
                m_filename = "config/config-mono.yaml";
                break;

            default:
                break;
            }

            m_root = YAML::LoadFile(m_filename);

            _updateInternalData();

            _printToStdout();
        }

        /**
         * Parse the configuration file and store the data internally
         * 
         * Since we are using a singleton, this avoids the extra memory usage of parsing the data at each cpp file required. Rather we
         * can call this instance and get the data as reference
         */
        void _updateInternalData()
        {
            DEBUG_LOG("Parsing configuration files ...");
            switch (ctype)
            {
            case GA:
            {
                m_genConfig.general.generations = m_root["Genetic-Algorithm"]["General"]["generations"].as<size_t>();
                m_genConfig.general.population_size = m_root["Genetic-Algorithm"]["General"]["population_size"].as<size_t>();
                m_genConfig.general.mating_pool_size = m_root["Genetic-Algorithm"]["General"]["mating_pool_size"].as<size_t>();
                m_genConfig.general.iterations_per_genome = m_root["Genetic-Algorithm"]["General"]["iterations_per_genome"].as<size_t>();
                m_genConfig.general.interactive_decision_tree = m_root["Genetic-Algorithm"]["General"]["interactive_decision_tree"].as<bool>();

                m_genConfig.operators.mutation_probability = m_root["Genetic-Algorithm"]["Operators"]["mutation_probability"].as<double>();
                m_genConfig.operators.crossover_bias = m_root["Genetic-Algorithm"]["Operators"]["crossover_bias"].as<double>();

                m_mpcConfigGA.general.timesteps = m_root["MPC-Controller"]["General"]["timesteps"].as<size_t>();
                m_mpcConfigGA.general.sample_time = m_root["MPC-Controller"]["General"]["sample_time"].as<double>();

                m_mpcConfigGA.initial_state.x = m_root["MPC-Controller"]["Initial-State"]["x"].as<double>();
                m_mpcConfigGA.initial_state.y = m_root["MPC-Controller"]["Initial-State"]["y"].as<double>();
                m_mpcConfigGA.initial_state.theta = m_root["MPC-Controller"]["Initial-State"]["theta"].as<double>();
                m_mpcConfigGA.initial_state.linear_velocity = m_root["MPC-Controller"]["Initial-State"]["linear-velocity"].as<double>();
                m_mpcConfigGA.initial_state.angular_velocity = m_root["MPC-Controller"]["Initial-State"]["angular-velocity"].as<double>();
                m_mpcConfigGA.initial_state.throttle = m_root["MPC-Controller"]["Initial-State"]["throttle"].as<double>();

                m_mpcConfigGA.desired.velocity = m_root["MPC-Controller"]["Desired"]["velocity"].as<double>();
                m_mpcConfigGA.desired.cross_track_error = m_root["MPC-Controller"]["Desired"]["cross_track_error"].as<double>();
                m_mpcConfigGA.desired.orientation_error = m_root["MPC-Controller"]["Desired"]["orientation_error"].as<double>();

                m_mpcConfigGA.max_bounds.omega = m_root["MPC-Controller"]["Max-Bounds"]["omega"].as<double>();
                m_mpcConfigGA.max_bounds.throttle = m_root["MPC-Controller"]["Max-Bounds"]["throttle"].as<double>();

                YAML::Node bounds = m_root["MPC-Controller"]["Weight-Bounds"];

                m_mpcConfigGA.weight_bounds.w_cte = std::make_pair(bounds["w_cte"][0].as<double>(), bounds["w_cte"][1].as<double>());
                m_mpcConfigGA.weight_bounds.w_etheta = std::make_pair(bounds["w_etheta"][0].as<double>(), bounds["w_etheta"][1].as<double>());
                m_mpcConfigGA.weight_bounds.w_vel = std::make_pair(bounds["w_vel"][0].as<double>(), bounds["w_vel"][1].as<double>());
                m_mpcConfigGA.weight_bounds.w_omega = std::make_pair(bounds["w_omega"][0].as<double>(), bounds["w_omega"][1].as<double>());
                m_mpcConfigGA.weight_bounds.w_acc = std::make_pair(bounds["w_acc"][0].as<double>(), bounds["w_acc"][1].as<double>());
                m_mpcConfigGA.weight_bounds.w_omega_d = std::make_pair(bounds["w_omega_d"][0].as<double>(), bounds["w_omega_d"][1].as<double>());
                m_mpcConfigGA.weight_bounds.w_acc_d = std::make_pair(bounds["w_acc_d"][0].as<double>(), bounds["w_acc_d"][1].as<double>());
                break;
            }
            case MONO:
            {
                m_mpcConfigMono.general.timesteps = m_root["MPC-Controller"]["General"]["timesteps"].as<size_t>();
                m_mpcConfigMono.general.sample_time = m_root["MPC-Controller"]["General"]["sample_time"].as<double>();

                m_mpcConfigMono.initial_state.x = m_root["MPC-Controller"]["Initial-State"]["x"].as<double>();
                m_mpcConfigMono.initial_state.y = m_root["MPC-Controller"]["Initial-State"]["y"].as<double>();
                m_mpcConfigMono.initial_state.theta = m_root["MPC-Controller"]["Initial-State"]["theta"].as<double>();
                m_mpcConfigMono.initial_state.linear_velocity = m_root["MPC-Controller"]["Initial-State"]["linear-velocity"].as<double>();
                m_mpcConfigMono.initial_state.angular_velocity = m_root["MPC-Controller"]["Initial-State"]["angular-velocity"].as<double>();
                m_mpcConfigMono.initial_state.throttle = m_root["MPC-Controller"]["Initial-State"]["throttle"].as<double>();

                m_mpcConfigMono.desired.velocity = m_root["MPC-Controller"]["Desired"]["velocity"].as<double>();
                m_mpcConfigMono.desired.cross_track_error = m_root["MPC-Controller"]["Desired"]["cross_track_error"].as<double>();
                m_mpcConfigMono.desired.orientation_error = m_root["MPC-Controller"]["Desired"]["orientation_error"].as<double>();

                m_mpcConfigMono.max_bounds.omega = m_root["MPC-Controller"]["Max-Bounds"]["omega"].as<double>();
                m_mpcConfigMono.max_bounds.throttle = m_root["MPC-Controller"]["Max-Bounds"]["throttle"].as<double>();

                m_mpcConfigMono.weights.w_cte = m_root["MPC-Controller"]["Weights"]["w_cte"].as<double>();
                m_mpcConfigMono.weights.w_etheta = m_root["MPC-Controller"]["Weights"]["w_etheta"].as<double>();
                m_mpcConfigMono.weights.w_vel = m_root["MPC-Controller"]["Weights"]["w_vel"].as<double>();
                m_mpcConfigMono.weights.w_omega = m_root["MPC-Controller"]["Weights"]["w_omega"].as<double>();
                m_mpcConfigMono.weights.w_acc = m_root["MPC-Controller"]["Weights"]["w_acc"].as<double>();
                m_mpcConfigMono.weights.w_omega_d = m_root["MPC-Controller"]["Weights"]["w_omega_d"].as<double>();
                m_mpcConfigMono.weights.w_acc_d = m_root["MPC-Controller"]["Weights"]["w_acc_d"].as<double>();

                m_mpcConfigMono.teardown_tolerance.velocity = m_root["MPC-Controller"]["Teardown-Tolerance"]["velocity"].as<double>();
                m_mpcConfigMono.teardown_tolerance.cross_track_error = m_root["MPC-Controller"]["Teardown-Tolerance"]["cross_track_error"].as<double>();
                m_mpcConfigMono.teardown_tolerance.orientation_error = m_root["MPC-Controller"]["Teardown-Tolerance"]["orientation_error"].as<double>();
                break;
            }
            default:
                break;
            }
        }

        /**
         * Print loaded parameters to STDOUT
         */
        void _printToStdout() const
        {
            switch (ctype)
            {
            case GA:
            {
                CONSOLE_LOG("* PARAMETERS  - Genetic Algorithm\n\n");
                CONSOLE_LOG("? Generations                  : " << m_genConfig.general.generations << std::endl);
                CONSOLE_LOG("? Population size              : " << m_genConfig.general.population_size << std::endl);
                CONSOLE_LOG("? Mating pool size             : " << m_genConfig.general.mating_pool_size << std::endl);
                CONSOLE_LOG("? Iterations per genome        : " << m_genConfig.general.iterations_per_genome << std::endl);
                CONSOLE_LOG("? Interactive Decision Tree    : " << m_genConfig.general.interactive_decision_tree << std::endl);
                CONSOLE_LOG("? Mutation probability         : " << m_genConfig.operators.mutation_probability << std::endl);
                CONSOLE_LOG("? Crossover bias               : " << m_genConfig.operators.crossover_bias << std::endl);
                CONSOLE_LOG(std::endl);
                CONSOLE_LOG("* PARAMETERS  - Model Predictive Control\n\n");
                CONSOLE_LOG("? Timesteps                    : " << m_mpcConfigGA.general.timesteps << std::endl);
                CONSOLE_LOG("? Sample time                  : " << m_mpcConfigGA.general.sample_time << std::endl);
                CONSOLE_LOG("? Initial state - x            : " << m_mpcConfigGA.initial_state.x << std::endl);
                CONSOLE_LOG("? Initial state - y            : " << m_mpcConfigGA.initial_state.y << std::endl);
                CONSOLE_LOG("? Initial state - theta        : " << m_mpcConfigGA.initial_state.theta << std::endl);
                CONSOLE_LOG("? Initial speed - linear       : " << m_mpcConfigGA.initial_state.linear_velocity << std::endl);
                CONSOLE_LOG("? Initial speed - angular      : " << m_mpcConfigGA.initial_state.angular_velocity << std::endl);
                CONSOLE_LOG("? Initial throttle             : " << m_mpcConfigGA.initial_state.throttle << std::endl);
                CONSOLE_LOG("? Desired velocity             : " << m_mpcConfigGA.desired.velocity << std::endl);
                CONSOLE_LOG("? Desired error - cross track  : " << m_mpcConfigGA.desired.cross_track_error << std::endl);
                CONSOLE_LOG("? Desired error - orientation  : " << m_mpcConfigGA.desired.orientation_error << std::endl);
                CONSOLE_LOG("? Constraints   - omega        : "
                            << "[ " << -m_mpcConfigGA.max_bounds.omega << ", " << m_mpcConfigGA.max_bounds.omega << " ]" << std::endl);
                CONSOLE_LOG("? Constraints   - throttle     : "
                            << "[ " << -m_mpcConfigGA.max_bounds.throttle << ", " << m_mpcConfigGA.max_bounds.throttle << " ]" << std::endl);
                CONSOLE_LOG("? Weight bounds - w_cte        : "
                            << "[ " << m_mpcConfigGA.weight_bounds.w_cte.first << ", " << m_mpcConfigGA.weight_bounds.w_cte.second << " ]" << std::endl);
                CONSOLE_LOG("? Weight bounds - w_etheta     : "
                            << "[ " << m_mpcConfigGA.weight_bounds.w_etheta.first << ", " << m_mpcConfigGA.weight_bounds.w_etheta.second << " ]" << std::endl);
                CONSOLE_LOG("? Weight bounds - w_vel        : "
                            << "[ " << m_mpcConfigGA.weight_bounds.w_vel.first << ", " << m_mpcConfigGA.weight_bounds.w_vel.second << " ]" << std::endl);
                CONSOLE_LOG("? Weight bounds - w_omega      : "
                            << "[ " << m_mpcConfigGA.weight_bounds.w_omega.first << ", " << m_mpcConfigGA.weight_bounds.w_omega.second << " ]" << std::endl);
                CONSOLE_LOG("? Weight bounds - w_acc        : "
                            << "[ " << m_mpcConfigGA.weight_bounds.w_acc.first << ", " << m_mpcConfigGA.weight_bounds.w_acc.second << " ]" << std::endl);
                CONSOLE_LOG("? Weight bounds - w_omega_d    : "
                            << "[ " << m_mpcConfigGA.weight_bounds.w_omega_d.first << ", " << m_mpcConfigGA.weight_bounds.w_omega_d.second << " ]" << std::endl);
                CONSOLE_LOG("? Weight bounds - w_acc_d      : "
                            << "[ " << m_mpcConfigGA.weight_bounds.w_acc_d.first << ", " << m_mpcConfigGA.weight_bounds.w_acc_d.second << " ]" << std::endl);
                CONSOLE_LOG(std::endl);
                break;
            }
            case MONO:
            {
                CONSOLE_LOG("* PARAMETERS  - Model Predictive Control\n\n");
                CONSOLE_LOG("? Timesteps                    : " << m_mpcConfigMono.general.timesteps << std::endl);
                CONSOLE_LOG("? Sample time                  : " << m_mpcConfigMono.general.sample_time << std::endl);
                CONSOLE_LOG("? Initial state - x            : " << m_mpcConfigMono.initial_state.x << std::endl);
                CONSOLE_LOG("? Initial state - y            : " << m_mpcConfigMono.initial_state.y << std::endl);
                CONSOLE_LOG("? Initial state - theta        : " << m_mpcConfigMono.initial_state.theta << std::endl);
                CONSOLE_LOG("? Initial speed - linear       : " << m_mpcConfigMono.initial_state.linear_velocity << std::endl);
                CONSOLE_LOG("? Initial speed - angular      : " << m_mpcConfigMono.initial_state.angular_velocity << std::endl);
                CONSOLE_LOG("? Initial throttle             : " << m_mpcConfigMono.initial_state.throttle << std::endl);
                CONSOLE_LOG("? Desired velocity             : " << m_mpcConfigMono.desired.velocity << std::endl);
                CONSOLE_LOG("? Desired error - cross track  : " << m_mpcConfigMono.desired.cross_track_error << std::endl);
                CONSOLE_LOG("? Desired error - orientation  : " << m_mpcConfigMono.desired.orientation_error << std::endl);
                CONSOLE_LOG("? Constraints   - omega        : "
                            << "[ " << -m_mpcConfigMono.max_bounds.omega << ", " << m_mpcConfigMono.max_bounds.omega << " ]" << std::endl);
                CONSOLE_LOG("? Constraints   - throttle     : "
                            << "[ " << -m_mpcConfigMono.max_bounds.throttle << ", " << m_mpcConfigMono.max_bounds.throttle << " ]" << std::endl);
                CONSOLE_LOG("? Weight        - w_cte        : " << m_mpcConfigMono.weights.w_cte << std::endl);
                CONSOLE_LOG("? Weight        - w_etheta     : " << m_mpcConfigMono.weights.w_etheta << std::endl);
                CONSOLE_LOG("? Weight        - w_vel        : " << m_mpcConfigMono.weights.w_vel << std::endl);
                CONSOLE_LOG("? Weight        - w_omega      : " << m_mpcConfigMono.weights.w_omega << std::endl);
                CONSOLE_LOG("? Weight        - w_acc        : " << m_mpcConfigMono.weights.w_acc << std::endl);
                CONSOLE_LOG("? Weight        - w_omega_d    : " << m_mpcConfigMono.weights.w_omega_d << std::endl);
                CONSOLE_LOG("? Weight        - w_acc_d      : " << m_mpcConfigMono.weights.w_acc_d << std::endl);
                CONSOLE_LOG(std::endl);
                break;
            }
            }
        }

        /**
         * Get the instance of the singleton
         * 
         * @return The instance
         */
        static ConfigHandler &get()
        {
            static ConfigHandler s_Instance;
            return s_Instance;
        }

        /************************* IMPLEMENTATIONS *************************/

        YAML::Node RootImpl()
        {
            return m_root;
        }

        template <ConfigType U = ctype>
        typename std::enable_if<U == 0, GeneticAlgorithm &>::type getGAConfigImpl()
        {
            return m_genConfig;
        }

        template <ConfigType U = ctype>
        typename std::enable_if<U == 0, MPC_Controller_GA &>::type getMpcConfigImpl()
        {
            return m_mpcConfigGA;
        }

        template <ConfigType U = ctype>
        typename std::enable_if<U == 1, MPC_Controller_MONO &>::type getMpcConfigImpl()
        {
            return m_mpcConfigMono;
        }
    };
} // namespace config

#endif
