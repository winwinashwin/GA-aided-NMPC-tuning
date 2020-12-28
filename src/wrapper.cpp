#include <boost/python.hpp>
#include <boost/python/list.hpp>
#include <boost/python/extract.hpp>
#include "mpc.h"

namespace bp = boost::python;

#define PYTHON_ERROR(TYPE, REASON)     \
    {                                  \
        PyErr_SetString(TYPE, REASON); \
        throw bp::error_already_set(); \
    }

template <class T>
inline PyObject *managingPyObject(T *p)
{
    return typename bp::manage_new_object::apply<T *>::type()(p);
}

template <class Copyable>
bp::object
generic__copy__(bp::object copyable)
{
    Copyable *newCopyable(new Copyable(bp::extract<const Copyable
                                                       &>(copyable)));
    bp::object
        result(bp::detail::new_reference(managingPyObject(newCopyable)));

    bp::extract<bp::dict>(result.attr("__dict__"))().update(
        copyable.attr("__dict__"));

    return result;
}

template <class Copyable>
bp::object
generic__deepcopy__(bp::object copyable, bp::dict memo)
{
    bp::object copyMod = bp::import("copy");
    bp::object deepcopy = copyMod.attr("deepcopy");

    Copyable *newCopyable(new Copyable(bp::extract<const Copyable
                                                       &>(copyable)));
    bp::object
        result(bp::detail::new_reference(managingPyObject(newCopyable)));

    // HACK: copyableId shall be the same as the result of id(copyable)
    intptr_t copyableId = (intptr_t)(copyable.ptr());
    memo[copyableId] = result;

    bp::extract<bp::dict>(result.attr("__dict__"))().update(
        deepcopy(bp::extract<bp::dict>(copyable.attr("__dict__"))(),
                 memo));

    return result;
}

struct State
{
    double x, y, theta;
    double v;
    double cte, etheta;

    State(double x,
          double y,
          double theta,
          double v,
          double cte,
          double etheta) : x(x),
                           y(y),
                           theta(theta),
                           v(v),
                           cte(cte),
                           etheta(etheta)
    {
    }
};

struct Solution
{
    double omega, throttle, cost;
};

class NMPC
{
public:
    NMPC(mpc::Params params, boost::python::list coeffs)
        : m_params(params)
    {
        const size_t l = len(coeffs);
        this->m_coeffs.resize(l);

        for (size_t i = 0; i < l; ++i)
            this->m_coeffs[i] = boost::python::extract<double>(coeffs[i]);
    }

    Solution solve(State s)
    {
        Eigen::VectorXd model_state(6);
        model_state << s.x, s.y, s.theta, s.v, s.cte, s.etheta;

        mpc::MPC _mpc(m_params, m_coeffs);

        std::vector<double> mpc_solns = _mpc.solve(model_state);

        Solution sol;
        sol.omega = mpc_solns[0];
        sol.throttle = mpc_solns[1];
        sol.cost = mpc_solns[2];
        return sol;
    }

private:
    Eigen::VectorXd m_coeffs;
    mpc::Params m_params;
};

BOOST_PYTHON_MODULE(nmpc)
{
    using namespace boost::python;

    class_<State>("State", init<double, double, double, double, double, double>())
        .def("__copy__", &generic__copy__<State>)
        .def("__deepcopy__", &generic__deepcopy__<State>)
        .def_readwrite("x", &State::x)
        .def_readwrite("y", &State::y)
        .def_readwrite("theta", &State::theta)
        .def_readwrite("v", &State::v)
        .def_readwrite("cte", &State::cte)
        .def_readwrite("etheta", &State::etheta);

    class_<Solution>("Solution")
        .add_property("omega", &Solution::omega)
        .add_property("throttle", &Solution::throttle)
        .add_property("cost", &Solution::cost);

    class_<NMPC>("NMPC", init<mpc::Params, boost::python::list>())
        .def("solve", &NMPC::solve);

    class_<mpc::__LH>("__LH")
        .def_readwrite("min", &mpc::__LH::min)
        .def_readwrite("max", &mpc::__LH::max);

    scope in_param = class_<mpc::Params>("Params", init<>())
                         .def_readwrite("forward", &mpc::Params::forward)
                         .def_readwrite("desired", &mpc::Params::desired)
                         .def_readwrite("limits", &mpc::Params::limits)
                         .add_property("BOUND_VALUE", &mpc::Params::BOUND_VALUE)
                         .def_readwrite("weights", &mpc::Params::weights);

    class_<mpc::Params::__Forward>("__Forward")
        .def_readwrite("timesteps", &mpc::Params::__Forward::timesteps)
        .def_readwrite("dt", &mpc::Params::__Forward::dt);

    class_<mpc::Params::__Desired>("__Desired")
        .def_readwrite("cte", &mpc::Params::__Desired::cte)
        .def_readwrite("etheta", &mpc::Params::__Desired::etheta)
        .def_readwrite("vel", &mpc::Params::__Desired::vel);

    class_<mpc::Params::__Limits>("__Limits")
        .def_readwrite("omega", &mpc::Params::__Limits::omega)
        .def_readwrite("throttle", &mpc::Params::__Limits::throttle);

    class_<mpc::Params::Weights>("Weights")
        .def_readwrite("vel", &mpc::Params::Weights::vel)
        .def_readwrite("cte", &mpc::Params::Weights::cte)
        .def_readwrite("etheta", &mpc::Params::Weights::etheta)
        .def_readwrite("omega", &mpc::Params::Weights::omega)
        .def_readwrite("acc", &mpc::Params::Weights::acc)
        .def_readwrite("omega_d", &mpc::Params::Weights::omega_d)
        .def_readwrite("acc_d", &mpc::Params::Weights::acc_d);
};