#include <sawConstraintController/testClass.h>

testClass::testClass() : mtsVFCartesianTranslation()
{

}

testClass::testClass(const std::string &name, mtsVFDataBase *data) : mtsVFCartesianTranslation(name,data) {
}
