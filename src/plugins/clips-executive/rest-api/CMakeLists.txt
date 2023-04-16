set(PLUGIN_clips-executive-rest-api
    ON
    CACHE BOOL ON)
if(PLUGIN_clips-executive-rest-api)
  add_library(
    clips-executive-rest-api MODULE
    clips-executive-rest-api.cpp
    clips-executive-rest-api-plugin.cpp
    model/Plan.cpp
    model/DomainEffect.cpp
    model/Goal.cpp
    model/GroundedPDDLFormula.cpp
    model/DomainObjectType.cpp
    model/GroundedPDDLPredicate.cpp
    model/DomainFact.cpp
    model/GroundedFormula.cpp
    model/DomainPredicate.cpp
    model/PDDLPredicate.cpp
    model/PlanAction.cpp
    model/PDDLFormula.cpp
    model/DomainObject.cpp
    model/DomainOperator.cpp
    model/PDDLGrounding.cpp
    model/DomainOperatorParameter.cpp)

  depend_on_webview(clips-executive-rest-api)

  target_link_libraries(clips-executive-rest-api fawkescore fawkesutils
                        fawkesaspects fawkesclipsaspect fawkeswebview)
  target_compile_features(clips-executive-rest-api PRIVATE cxx_std_17)
else()
  plugin_disabled_message(clips-executive-rest-api)
endif()
