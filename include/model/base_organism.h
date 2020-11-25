#ifndef MODEL_BASE_ORG_H_
#define MODEL_BASE_ORG_H_

#include "primary.h"
#include "model/differential_drive.h"
#include "mpc_lib/mpc.h"
#include "utils/config_handler.hpp"
#include "utils/json_logger.hpp"

/**
 * Models used
 */
namespace model
{
    /**
     * Structure representing the terminating condition of the control loop for 
     * each mode
     * 
     * @tparam __type: The mode of operation, config::MONO or config::GA
     */
    template <config::ConfigType __type>
    struct TerminateOn;

    template <>
    struct TerminateOn<config::MONO>
    {
        struct __Tol
        {
            double vel, cte, etheta;
        } tolerance;
    };

    template <>
    struct TerminateOn<config::GA>
    {
        size_t iterations;
    };

    template <config::ConfigType __type>
    class BaseOrganism
    {
    public:
        /**
         * Set the initial state of the internal model
         * 
         * @param s: Initial state
         */
        void setModelInitState(const model::State &s)
        {
            m_dModel.setInitState(s);
        }

        /**
         * Get performance/response data of the organism in the control loop
         * 
         * @return The performance data
         */
        model::Performance getPerformance() const
        {
            return m_performance;
        }

        /**
         * Refresh/reset the organism
         * 
         * (i)   Bring the internal differential drive model back to the initial state
         * (ii)  Clear pervious performance data
         * (iii) Reset the logger
         */
        void refresh()
        {
            m_dModel.reset();
            m_performance.reset();
            m_jsonLogger = JsonLogger();
        }

        /**
         * Follow the setpoints
         * 
         * This is the main control loop
         * 
         * @return True on successful run, false otherwise
         */
        [[ nodiscard ]] bool followSetpoints(const mpc::Params &params, const TerminateOn<__type> &term);

    private:
        /// Internal differential drive model
        model::DifferentialDrive m_dModel;

        /// Performance data
        model::Performance m_performance;

    protected:
        JsonLogger m_jsonLogger;
    };
} // namespace model
#endif
