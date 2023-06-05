using RAWSimO.Core.Configurations;
using RAWSimO.Core.Control.Defaults.PathPlanning;
using RAWSimO.Toolbox;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RAWSimO.Core.Control.Defaults.PathPlanning
{


    /// <summary>
    /// Declares a scheduled method manager that changes methods at specified time points to the ones supplied by the configuration.
    /// </summary>
    public class HyperHeuristicsPathManager : PathManager
    {
        /// <summary>
        /// Creates a new instance of the manager.
        /// </summary>
        /// <param name="instance">The instance this manager belongs to.</param>
        public HyperHeuristicsPathManager(Instance instance) : base(instance)
        {
            _config = instance.ControllerConfig.PathPlanningConfig as HyperHeuristicsPathManagerConfiguration;
            // Set first change event
            _PathManagerQueue = new Queue<Skvp<double, PathPlanningMethodType>>(_config.HHPPManagers);
        }

        /// <summary>
        /// The config of this controller.
        /// </summary>
        private HyperHeuristicsPathManagerConfiguration _config;
        /// <summary>
        /// All switches that need to be done for pod storage management.
        /// </summary>
        private Queue<Skvp<double, PathPlanningMethodType>> _PathManagerQueue;
        /// <summary>
        /// Translates the time depending on the setting for relative mode.
        /// </summary>
        /// <param name="time">The time to translate.</param>
        /// <returns>The time in simulation time.</returns>
        private double TranslateTime(double time) { return _config.RelativeMode ? time * (Instance.SettingConfig.SimulationWarmupTime + Instance.SettingConfig.SimulationDuration) : time; }

        /// <summary>
        /// The next event when this element has to be updated.
        /// </summary>
        /// <param name="currentTime">The current time of the simulation.</param>
        /// <returns>The next time this element has to be updated.</returns>
        public override double GetNextEventTime(double currentTime) { return _PathManagerQueue.Any() ? TranslateTime(_PathManagerQueue.First().Key) : double.PositiveInfinity; }
        ///En la linea anterior cambie de public override double a public virtual double
        /// <summary>0
        /// Updates the element to the specified time.
        /// </summary>
        /// <param name="lastTime">The time before the update.</param>
        /// <param name="currentTime">The time to update to.</param>
        public override void Update(double lastTime, double currentTime)
        {
            // Check whether a change has to happen
            if (_PathManagerQueue.Any() && currentTime > TranslateTime(_PathManagerQueue.First().Key))
            {
                // --> Handle pod storage managers
                // Get next manager
                PathPlanningMethodType nextPathManagerType = _PathManagerQueue.Dequeue().Value;
                PathManager newPathManager;
                // Prepare it
                switch (nextPathManagerType)
                {
                    case PathPlanningMethodType.Dummy:
                        {
                            Instance.ControllerConfig.PathPlanningConfig = new DummyPathPlanningConfiguration();
                            newPathManager = new DummyPathManager(Instance);
                        }
                        break;
                    case PathPlanningMethodType.WHCAvStar:
                        {
                            Instance.ControllerConfig.PathPlanningConfig = new WHCAvStarPathPlanningConfiguration();
                            newPathManager = new WHCAvStarPathManager(Instance);
                        }
                        break;
                    case PathPlanningMethodType.WHCAnStar:
                        {
                            Instance.ControllerConfig.PathPlanningConfig = new WHCAnStarPathPlanningConfiguration();
                            newPathManager = new WHCAnStarPathManager(Instance);
                        }
                        break;
                    case PathPlanningMethodType.FAR:
                        {
                            Instance.ControllerConfig.PathPlanningConfig = new FARPathPlanningConfiguration();
                            newPathManager = new FARPathManager(Instance);
                        }
                        break;
                    case PathPlanningMethodType.CBS:
                        {
                            Instance.ControllerConfig.PathPlanningConfig = new CBSPathPlanningConfiguration();
                            newPathManager = new CBSPathManager(Instance);
                        }
                        break;
                    case PathPlanningMethodType.BCP:
                        {
                            Instance.ControllerConfig.PathPlanningConfig = new BCPPathPlanningConfiguration();
                            newPathManager = new BCPPathManager(Instance);
                        }
                        break;
                    case PathPlanningMethodType.PAS:
                        {
                            Instance.ControllerConfig.PathPlanningConfig = new PASPathPlanningConfiguration();
                            newPathManager = new PASPathManager(Instance);
                        }
                        break;
                    case PathPlanningMethodType.OD_ID:
                        {
                            Instance.ControllerConfig.PathPlanningConfig = new ODIDPathPlanningConfiguration();
                            newPathManager = new ODIDPathManager(Instance);
                        }
                        break;
                    case PathPlanningMethodType.Simple: throw new ArgumentException("Cannot switch to fixed mechanism, because the system is already running!");
                    default: throw new ArgumentException("Unknown path planning manager: " + nextPathManagerType);
                }
                // Change it
                Instance.Controller.ExchangePathManager(newPathManager);
            }
        }
    }
}
