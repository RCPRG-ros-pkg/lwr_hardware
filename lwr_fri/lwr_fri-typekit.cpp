#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/BoostArrayTypeInfo.hpp>
#include <boost/array.hpp>

namespace lwr_fri {
  using namespace RTT;

    /**
     * This interface defines the types of the realTime package.
     */
    class lwr_fri_TypekitPlugin
      : public types::TypekitPlugin
    {
    public:
      virtual std::string getName(){
          return std::string("lwr_fri_typekit");
      }

      virtual bool loadTypes() {
	RTT::types::Types()->addType(new types::BoostArrayTypeInfo<boost::array<float,7> >("float32[7]"));
	return true;
      }
      virtual bool loadOperators() { return true; }
      virtual bool loadConstructors() { return true; }
    };
}

ORO_TYPEKIT_PLUGIN( lwr_fri::lwr_fri_TypekitPlugin )
