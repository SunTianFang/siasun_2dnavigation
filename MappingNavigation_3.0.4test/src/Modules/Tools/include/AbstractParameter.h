#pragma once

#include <string>
#include <map>
#include <vector>
#include <iostream>
#include <string.h>


/**
 * Helper defines for std iterator loops
 */
#define forEach( listtype, list ) \
    for ( listtype::iterator iter = (list)->begin(); iter != (list)->end(); ++iter )

#define forEachAs( listtype, list, iter ) \
    for ( listtype::iterator iter = (list)->begin(); iter != (list)->end(); ++iter )

#define const_forEach( listtype, list ) \
    for ( listtype::const_iterator iter = (list)->begin(); iter != (list)->end(); ++iter )

#define const_forEachAs( listtype, list, iter ) \
    for ( listtype::const_iterator iter = (list)->begin(); iter != (list)->end(); ++iter )

#define forEachR( listtype, list ) \
    for ( listtype::reverse_iterator iter = (list)->rbegin(); iter != (list)->rend(); ++iter )

#define const_forEachR( listtype, list ) \
    for ( listtype::const_reverse_iterator iter = (list)->rbegin(); iter != (list)->rend(); ++iter )

////////////////////////////////////////////////////////////////////////////////////////
namespace robo
{
/**
   * Subclass this class to make a non-copyable class (copy
   * constructor and assignment operator are private)
   */
class NonCopyable
{
private:
    NonCopyable(const NonCopyable&);
    const NonCopyable& operator=(const NonCopyable&);

protected:
    NonCopyable()
    {
    }

    virtual ~NonCopyable()
    {
    }
};  // class NonCopyable

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

class AbstractParameter;

/**
   * Type declaration of AbstractParameter vector
   */
typedef std::vector<AbstractParameter*> ParameterVector;

/**
   * Parameter manager.
   */
class ParameterManager : public NonCopyable
{
public:
    /**
     * Default constructor
     */
    ParameterManager()
    {
    }

    /**
     * Destructor
     */
    virtual ~ParameterManager()
    {
        Clear();
    }

public:
    /**
     * Adds the parameter to this manager
     * @param pParameter
     */
    void Add(AbstractParameter* pParameter);

    /**
     * Gets the parameter of the given name
     * @param rName
     * @return parameter of given name
     */
    AbstractParameter* Get(const std::string& rName)
    {
        if (m_ParameterLookup.find(rName) != m_ParameterLookup.end())
        {
            return m_ParameterLookup[rName];
        }

        std::cout << "Unknown parameter: " << rName << std::endl;

        return NULL;
    }

    /**
     * Clears the manager of all parameters
     */
    void Clear();

    /**
     * Gets all parameters
     * @return vector of all parameters
     */
    inline const ParameterVector& GetParameterVector() const
    {
        return m_Parameters;
    }

public:
    /**
     * Gets the parameter with the given name
     * @param rName
     * @return parameter of given name
     */
    AbstractParameter* operator() (const std::string& rName)
    {
        return Get(rName);
    }

private:
    ParameterManager(const ParameterManager&);
    const ParameterManager& operator=(const ParameterManager&);

private:
    ParameterVector m_Parameters;
    std::map<std::string, AbstractParameter*> m_ParameterLookup;
};  // ParameterManager

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
   * Abstract base class for robot objects.
   */
class Object : public NonCopyable
{
public:
    /**
     * Default constructor
     */
    Object();

    /**
     * Default constructor
     */
    virtual ~Object();

public:
    /**
     * Gets the class name of this object
     * @return class name
     */
    virtual const char* GetClassName() const {return "";}

    /**
     * Gets the type of this object
     * @return object type
     */
    virtual unsigned int GetObjectType() const {return 0;}

    /**
     * Gets the parameter manager of this dataset
     * @return parameter manager
     */
    virtual inline ParameterManager* GetParameterManager()
    {
        return m_pParameterManager;
    }

    /**
     * Add the parameter with the given name with the given value
     * @param rName name
     * @param value value
     */
    template<typename T>
    inline void AddParameter(const std::string& rName, T value);

    /**
     * Add the parameter
     * @param parameter
     */
    inline void AddParameter(AbstractParameter* pParameter)
    {
        if(NULL != m_pParameterManager){
            m_pParameterManager->Add(pParameter);
        }
    }

    /**
     * Gets the named parameter
     * @param rName name of parameter
     * @return parameter
     */
    inline AbstractParameter* GetParameter(const std::string& rName) const
    {
        return m_pParameterManager->Get(rName);
    }

    /**
     * Sets the parameter with the given name with the given value
     * @param rName name
     * @param value value
     */
    template<typename T>
    inline void SetParameter(const std::string& rName, T& value);

    /**
     * Gets all parameters
     * @return parameters
     */
    inline const ParameterVector& GetParameters() const
    {
        return m_pParameterManager->GetParameterVector();
    }

    /**
     * Get the parameter value with the given name
     * @param rName name
     * @return value value
     */
    template<typename T>
    inline bool GetParameterValue(const std::string& rName, T& value);

private:
    Object(const Object&);
    const Object& operator=(const Object&);

private:
    ParameterManager* m_pParameterManager;
};

/**
   * Type declaration of Object vector
   */
typedef std::vector<Object*> ObjectVector;


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
   * Abstract base class for Parameters
   */
class AbstractParameter
{
public:
    /**
     * Constructs a parameter with the given name
     * @param rName
     * @param pParameterManger
     */
    AbstractParameter(const std::string& rName, ParameterManager* pParameterManger = NULL)
        : m_Name(rName)
    {
        // if parameter manager is provided add myself to it!
        if (pParameterManger != NULL)
        {
            pParameterManger->Add(this);
        }
    }

    /**
     * Constructs a parameter with the given name and description
     * @param rName
     * @param rDescription
     * @param pParameterManger
     */
    AbstractParameter(const std::string& rName,
                      const std::string& rDescription,
                      ParameterManager* pParameterManger = NULL)
        : m_Name(rName)
        , m_Description(rDescription)
    {
        // if parameter manager is provided add myself to it!
        if (pParameterManger != NULL)
        {
            pParameterManger->Add(this);
        }
    }

    /**
     * Destructor
     */
    virtual ~AbstractParameter()
    {
    }

public:
    /**
     * Gets the name of this object
     * @return name
     */
    inline const std::string& GetName() const
    {
        return m_Name;
    }

    /**
     * Returns the parameter description
     * @return parameter description
     */
    inline const std::string& GetDescription() const
    {
        return m_Description;
    }

    /**
     * Get parameter value as string.
     * @return value as string
     */
    virtual const std::string GetValueAsString() const = 0;

    /**
     * Set parameter value from string.
     * @param rStringValue value as string
     */
    virtual void SetValueFromString(const std::string& rStringValue) = 0;

    /**
     * Clones the parameter
     * @return clone
     */
    virtual AbstractParameter* Clone() = 0;

public:
    /**
     * Write this parameter onto output stream
     * @param rStream output stream
     * @param rParameter
     */
    friend std::ostream& operator << (std::ostream& rStream, const AbstractParameter& rParameter)
    {
        rStream.precision(6);
        rStream.flags(std::ios::fixed);

        rStream << rParameter.GetName() << " = " << rParameter.GetValueAsString();
        return rStream;
    }

private:
    std::string m_Name;
    std::string m_Description;
};  // AbstractParameter

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
   * Parameter class
   */
template<typename T>
class Parameter : public AbstractParameter
{
public:
    /**
     * Parameter with given name and value
     * @param rName
     * @param value
     * @param pParameterManger
     */
    Parameter(const std::string& rName, T value, ParameterManager* pParameterManger = NULL)
        : AbstractParameter(rName, pParameterManger)
        , m_Value(value)
    {
    }

    /**
     * Parameter with given name, description and value
     * @param rName
     * @param rDescription
     * @param value
     * @param pParameterManger
     */
    Parameter(const std::string& rName,
              const std::string& rDescription,
              T value,
              ParameterManager* pParameterManger = NULL)
        : AbstractParameter(rName, rDescription, pParameterManger)
        , m_Value(value)
    {
    }

    /**
     * Destructor
     */
    virtual ~Parameter()
    {
    }

public:
    /**
     * Gets value of parameter
     * @return parameter value
     */
    inline const T& GetValue() const
    {
        return m_Value;
    }

    /**
     * Sets value of parameter
     * @param rValue
     */
    inline void SetValue(const T& rValue)
    {
        m_Value = rValue;
    }

    /**
     * Get parameter value as string.
     * @return value as string
     */
    virtual const std::string GetValueAsString() const {return "";}

    /**
     * Set parameter value from string.
     * @param rStringValue value as string
     */
    virtual void SetValueFromString(const std::string& rStringValue) {}

    /**
     * Clone this parameter
     * @return clone of this parameter
     */
    virtual Parameter* Clone()
    {
        return new Parameter(GetName(), GetDescription(), GetValue());
    }

public:
    /**
     * Assignment operator
     */
    Parameter& operator = (const Parameter& rOther)
    {
        m_Value = rOther.m_Value;

        return *this;
    }

    /**
     * Sets the value of this parameter to given value
     */
    T operator = (T value)
    {
        m_Value = value;

        return m_Value;
    }

protected:
    /**
     * Parameter value
     */
    T m_Value;
};  // Parameter


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

// @cond EXCLUDE

template<typename T>
inline void Object::SetParameter(const std::string& rName, T& value)
{
    Parameter<T>* pParameter = dynamic_cast<Parameter<T>*>(GetParameter(rName));
    if (pParameter != NULL)
    {
        pParameter->SetValue(value);
    }
    else
    {
        std::cout << "Parameter does not exist: " << rName << std::endl;
    }
}

template<typename T>
inline void Object::AddParameter(const std::string& rName, T value)
{
    if(GetParameter(rName) != NULL){
        std::cout << "This parameter exist: " << rName << std::endl;
        return;
    }

   Parameter<T>* pParameter = new Parameter<T>(rName, value ,GetParameterManager());
}

template<typename T>
inline bool Object::GetParameterValue(const std::string& rName, T& value)
{
    Parameter<T>* pParameter = dynamic_cast<Parameter<T>*>(GetParameter(rName));
    if (pParameter != NULL){
        value = pParameter->GetValue();
        return true;
    }
    else{
        std::cout << "This parameter does not exist: " << rName << std::endl;
        return false;
    }
}

}  // namespace robo

