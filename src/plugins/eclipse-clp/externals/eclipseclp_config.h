#include <config/config.h>

namespace fawkes
{
class EclExternalConfig
{
private:
  /** Constructor. */
  EclExternalConfig();

  /** Constructor.
   * @param config config instance to use
   */
  EclExternalConfig(Configuration *config);
public:
  /** Destructor. */
  ~EclExternalConfig();

  static void create_initial_object(Configuration *config);
  static EclExternalConfig* instance();

  static Configuration* config_instance();

private:
  static EclExternalConfig *          m_instance;
  static Configuration *              m_config;
};
}

extern "C" int p_get_config_value();