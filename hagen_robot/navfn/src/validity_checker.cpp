#include "navfn/validity_checker.h"

namespace reeds_shepp
{
      ValidityChecker::ValidityChecker(const ompl::base::SpaceInformationPtr& si) : ompl::base::StateValidityChecker(si) {

      }

      bool ValidityChecker::isValid(const ompl::base::State* state){
          return this->clearance(state) > 0.0;
      }

      double ValidityChecker::clearance(const ompl::base::State* state) const {
           const ompl::base::RealVectorStateSpace::StateType* state2D =
              state->as<ompl::base::RealVectorStateSpace::StateType>();
          double x = state2D->values[0];
          double y = state2D->values[1];
          return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25;
      }
}