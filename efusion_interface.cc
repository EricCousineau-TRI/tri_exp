#include "efusion_interface.h"

EFusionInterface::EFusionInterface() {
	efusion_ = std::make_unique<ElasticFusion>(); 
}



