#include <assert.h>
#include "AbstractParameter.h"

namespace robo
{

Object::Object()
    : m_pParameterManager(new ParameterManager())
{
}

Object::~Object()
{
    if(NULL != m_pParameterManager){
        delete m_pParameterManager;
        m_pParameterManager = NULL;
    }
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void ParameterManager::Clear()
{
    forEach(robo::ParameterVector, &m_Parameters)
    {
        delete *iter;
    }

    m_Parameters.clear();
    m_ParameterLookup.clear();
}

void ParameterManager::Add(AbstractParameter* pParameter)
{
    if (pParameter != NULL && pParameter->GetName() != "")
    {
        if (m_ParameterLookup.find(pParameter->GetName()) == m_ParameterLookup.end())
        {
            m_Parameters.push_back(pParameter);

            m_ParameterLookup[pParameter->GetName()] = pParameter;
        }
        else
        {
            //m_ParameterLookup[pParameter->GetName()]->SetValueFromString(pParameter->GetValueAsString());
            std::cout << "Parameter exist: " << pParameter->GetName() << std::endl;

            //assert(false);
        }
    }
}

}  // namespace robo
