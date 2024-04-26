#ifndef CDIAGNOSIS_H
#define CDIAGNOSIS_H

class CDiagnosis
{
public:
    bool use_core_dump;
    bool auto_save_log;
    bool publish_point_cloud;
    bool publish_ndt_cell;
    bool publish_reflector;

public:
    CDiagnosis();

    ~CDiagnosis()
    {
    }

    CDiagnosis(const CDiagnosis& other)
    {
        this->use_core_dump = other.use_core_dump;
        this->auto_save_log = other.auto_save_log;
        this->publish_point_cloud = other.publish_point_cloud;
        this->publish_ndt_cell = other.publish_ndt_cell;
        this->publish_reflector = other.publish_reflector;
    }

    CDiagnosis& operator = (const CDiagnosis& Obj)
    {
        this->use_core_dump = Obj.use_core_dump;
        this->auto_save_log = Obj.auto_save_log;
        this->publish_point_cloud = Obj.publish_point_cloud;
        this->publish_ndt_cell = Obj.publish_ndt_cell;
        this->publish_reflector = Obj.publish_reflector;
        return *this;
    }

    void Initialize()
    {
        use_core_dump = false;
        auto_save_log = false;
        publish_point_cloud = false;
        publish_ndt_cell = false;
        publish_reflector = false;
    }
};
#endif // CDIAGNOSIS_H
