/* ============================================================================================================================ *//**
 * @file       master.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 26th May 2022 1:22:28 pm
 * @modified   Friday, 1st July 2022 1:22:34 pm
 * @project    engineering-thesis
 * @brief      Definition of the Master class providing API entry for implementing hardware-specific drivers of EtherCAT
 *             master devices
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_MASTER_H__
#define __CIFX_ETHERCAT_MASTER_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <chrono>
// Private includes
#include "cifx.hpp"
#include "ethercat.hpp"
#include "cifx/ethercat/slave.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx::ethercat {

/* ============================================================= Class ============================================================ */

/**
 * @brief Implementation of the @ref ::ethercat::Master driver with the CIFX/netX
 *   C++ Toolkit hardware access layer
 * 
 * @note Due to actual implementation of the CIFX EtherCAt master firmware basename of the ENI file, that is
 *    configured for the device whose channel is used to construct the Master driver, must be exactly 'ethercat.xml'. 
 *    This is NOT explicitly written in Hillschers documentation. Failing to follow this rule will result in strange
 *    errors during system bootup.
 */
class Master : public ::ethercat::Master<Master, Slave> {

    /// Make base class a friend to let it access implementation methods
    friend class ::ethercat::Master<Master, Slave>;

public: /* -------------------------------------------------- Public constants ---------------------------------------------------- */
    
    /**
     * @brief Tag of the data area of the CIFX device's channel used for EtherCAT communication
     * @details The @ref Matser driver utilizes onyl the 0'th area of the CIFX channel for EtherCAT
     *    communication. This is a 'regular' channel ( as opposed to 'priority' channel with idnex 
     *    @c 1 ). This is a default area used for communciation by CIFX EtherCAT firmware.
     * 
     * @see 'cifx_toolkit/doc/netx Dual-Port Memory Interface DPM 17 EN.pdf' (p.118/154)
     */
    static constexpr cifx::ProcessData::Area CIFX_PDI_DATA_AREA = cifx::ProcessData::Area::Regular;
    
public: /* ---------------------------------------------------- Public types ------------------------------------------------------ */

    /// Type of the underlying Master interface
    using MasterInterfaceT = ::ethercat::Master<Master, Slave>;
    
    /**
     * @brief Enume of possible states of the Master in the ESM (EtherCAT State Machine)
     *    extended with CIFX-specific states of the interface device
     */
    enum class ExtendedState : std::underlying_type_t<State>{
        Init   = static_cast<std::underlying_type_t<State>>(State::Init),
        Preop  = static_cast<std::underlying_type_t<State>>(State::Preop),
        Safeop = static_cast<std::underlying_type_t<State>>(State::Safeop),
        Op     = static_cast<std::underlying_type_t<State>>(State::Op),
        Busoff,
        LeaveOp,
        Busscan,
        BusscanComplete,
    };

    /**
     * @brief Structure describing CIFX-specific state of the Master interface
     */
    struct StateInfo {

        /// Currently requested target state
        ExtendedState target_state;
        /// Current state
        ExtendedState current_state;

        /// Stop reason ( @c 0 if no error )
        uint32_t stop_reason;

        /// Additional flags
        struct {
            
            /// If this flag is set, at least one mandatory slave is not in OP when master is in OP. But, the slave is still connected
            bool at_least_one_mandatory_slave_not_in_op;
            /// If this flag is set, the DC handling stopped sending ARMW/FRMW telegrams. The DC Slaves are not synchronizing their sys time in that case.
            bool dc_xrmw_stopped;
            /// If this flag is set, at least one mandatory slave is not connected to master anymore
            bool at_least_one_mandatory_slave_lost;

        } flags;

    };

public: /* --------------------------------------------- Public CIFX-specific types ----------------------------------------------- */

    /**
     * @brief Enumeration of host-bus synchronisation modes 
     * @see 'doc/EtherCAT Master V4 Protocol API 06 EN.pdf'
     */
    enum class SyncMode {
        FreeRun,
        IO1,
        IO2
    };

    /**
     * @brief Description of timing parameters of the bus
     */
    struct TimingInfo {

        /// Duration of the bus cycle
        std::chrono::nanoseconds bus_cycle;
        /// Duration of the frame transmition
        std::chrono::nanoseconds frame_transmition_time;
        /// Expected bus delay
        std::chrono::nanoseconds expected_bus_delay;
        /// Expected time of the RX transaction end (from start of bus cycle transmission)
        std::chrono::nanoseconds expected_rx_end_time; 
        /// Expected time of the TX transaction end (from start of bus cycle transmission)
        std::chrono::nanoseconds expected_tx_end_time; 
        
    };
    
public: /* ------------------------------------------------ Public static methods ------------------------------------------------- */

    /// Forward State-conversion function
    using MasterInterfaceT::state_to_str;

    /**
     * @brief Converts @p state to the human-readable string
     * 
     * @param state 
     *    state to be converted
     * @returns 
     *    human-readable name of the state
     */
    static constexpr std::string_view state_to_str(ExtendedState state);

public: /* -------------------------------------------------- Public ctors & dtors ------------------------------------------------ */

    /**
     * @brief Creates EtherCAT master driver operating on the given @p channel of the
     *    CIFX interface
     * 
     * @param channel 
     *    channel to be used for EtherCAT communication
     */
    Master(cifx::Channel &channel);

    /// Disabled copy semantic
    Master(const Master &rmaster) = delete;
    Master &operator=(const Master &rmaster) = delete;
    /// Disabled moving semantic
    Master(Master &&rmaster) = delete;
    Master &operator=(Master &&rmaster) = delete;

public: /* ------------------------------------------- Public EtherCAT common methods --------------------------------------------- */

    /**
     * @brief Reads current state of the master device in the ESM (EtherCAT slave machine)
     *   along with CIFX-specific informations
     * 
     * @param timeout 
     *    access timeout
     * @returns 
     *    current state of the slave device in the ESM (EtherCAT slave machine)
     * 
     * @throws cifx::Error 
     *    on error
     * @throws std::range_error
     *    if invalid state identifier has been returned by the CIFX device
     */
    StateInfo get_state_info(std::chrono::milliseconds timeout = std::chrono::milliseconds{ 100 }) const;

public: /* ------------------------------------- Public CIFX-specific methods (bus control) -------------------------------------- */

    /**
     * @brief Sets target host-bus synchronisation mode
     * 
     * @param mode 
     *    target synchronisation mode
     * @param timeout 
     *    operation's timeout
     * 
     * @throws cifx::Error 
     *    on error
     */
    void set_sync_mode(SyncMode mode, std::chrono::milliseconds timeout = std::chrono::seconds{ 5 });

    /**
     * @brief Measures current timing parameters of the bus
     * 
     * @param timeout 
     *    measurement timeout
     * @returns 
     *    measured timing parameters
     * 
     * @throws cifx::Error 
     *    on error
     */
    TimingInfo get_timing_info(std::chrono::milliseconds timeout = std::chrono::seconds{ 5 }) const;

public: /* ---------------------------------- Public CIFX-specific methods (comm-area mapping) ----------------------------------- */

    /**
     * @brief CIFX-specific structure holding information about how subsequent regions of Communication Channel I/O Areas
     *    are used by the driver (i.e. what kind of data is mapped into the regions)
     * 
     * @see 'doc/EtherCAT Master V4 Protocol API 06 EN.pdf', p. 149-151
     */
    struct CyclicMappingInfo {

        /// Mappin info entry for RX direction
        struct RxEntry {

            /**
             * @brief Type of data mapped into the region of the Communication Channel Input I/O Area
             */
            enum class Type {
                
                // No process data mapping for given direction
                Unused = VAL_ECM_IF_CYCLIC_CMD_DATATYPE_UNUSED,        
                // Process data is mapped in given direction
                ProcessData = VAL_ECM_IF_CYCLIC_CMD_DATATYPE_PROCESS_DATA,        
                // DC SysTime is mapped (only valid for receive direction)
                DcSystime = VAL_ECM_IF_CYCLIC_CMD_DATATYPE_DC_SYSTIME,        
                // Ored ALSTATUS of all slaves (only valid for receive direction)
                BrdAlstatus = VAL_ECM_IF_CYCLIC_CMD_DATATYPE_BRD_ALSTATUS,        
                // Ored DcSysTimeDifference register of all slaves (only valid for receive direction)
                BrdDcSystimeDiff = VAL_ECM_IF_CYCLIC_CMD_DATATYPE_BRD_DC_SYSTIME_DIFF,        
                // Area of WcState Bits (only valid for receive direction)
                WCStateBits = VAL_ECM_IF_CYCLIC_CMD_DATATYPE_WCSTATE_BITS,        
                // Area of ExtSync Status (only valid for receive direction)
                ExtsyncStatus = VAL_ECM_IF_CYCLIC_CMD_DATATYPE_EXTSYNC_STATUS
            
            };

            /**
             * @returns 
             *    human-readable name of the mapping type
             */
            static constexpr auto type_to_str(Type type);

            /// Type of data mapped in the region
            Type type;
            /// Offset of the region in the Area in [B]
            uint16_t offset;
            /// Size of the region in [B]
            uint16_t size;

            /// Placement of received Working Counter (WKC) in receive process data image (if present)
            std::optional<uint16_t> wkc_compare_received_byte_offset;

        };

        /// Mappin info entry for TX direction
        struct TxEntry {

            /**
             * @brief Type of data mapped into the region of the Communication Channel Output I/O Area
             */
            enum class Type {
                
                // No process data mapping for given direction
                Unused = VAL_ECM_IF_CYCLIC_CMD_DATATYPE_UNUSED,        
                // Process data is mapped in given direction
                ProcessData = VAL_ECM_IF_CYCLIC_CMD_DATATYPE_PROCESS_DATA
            
            };

            /**
             * @returns 
             *    human-readable name of the mapping type
             */
            static constexpr auto type_to_str(Type type);

            /// Type of data mapped in the region
            Type type;
            /// Offset of the region in the Area in [B]
            uint16_t offset;
            /// Size of the region in [B]
            uint16_t size;

        };

        /// Descriptors of regions in the RX (Input) I/O Are
        std::vector<RxEntry> rx;
        /// Descriptors of regions in the TX (Output) I/O Are
        std::vector<TxEntry> tx;

        /**
         * @brief Converts mapping info into the human-readable string
         */
        std::string to_str() const;

    };

    /**
     * @brief Reads informations about mapping of regions of Cyclic Data Areas of the Communciation channel 
     *    used by the CIFX Firmware
     * 
     * @param timeout 
     *     operation's timeout
     *    
     * @see 'doc/EtherCAT Master V4 Protocol API 06 EN.pdf', p. 149-151
     */
    CyclicMappingInfo get_cyclic_mapping_info(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{ 1000 }
    ) const;

protected: /* -------------------------------- Protected EtherCAT common methods (implementation) --------------------------------- */

    /**
     * @brief Reads current state of the slave device in the ESM (EtherCAT slave machine)
     * 
     * @param timeout 
     *    access timeout
     * @returns 
     *    current state of the slave device in the ESM (EtherCAT slave machine)
     * 
     * @throws Error 
     *    on error
     * @throws std::range_error
     *    if invalid state identifier has been returned by the CIFX device
     */
    State get_state_impl(std::chrono::milliseconds timeout) const;
    
    /**
     * @brief Requestes state change of the slave device in the ESM (EtherCAT slave machine)
     * 
     * @param state 
     *    target state
     * @param timeout 
     *    access timeout
     * 
     * @throws Error 
     *    on error
     * @throws std::range_error
     *    if invalid state has been requested
     */
    void set_state_impl(State state, std::chrono::milliseconds timeout);
    
protected: /* -------------------------------- Protected EtherCAT I/O methods (implementation) ------------------------------------ */

    /**
     * @brief Reads Input Process Data Image from the bus (secondly updates slave's input PDOs)
     * 
     * @param[out] pdi_buffer
     *    PDI buffer to read data to
     * @param[in] timeout
     *    timeout of the I/O operation
     * 
     * @throws cifx::Error
     *    on failure
     */
    inline void read_bus_impl(::ethercat::config::types::Span<uint8_t> pdi_buffer, std::chrono::milliseconds timeout);

    /**
     * @brief Writes Output Process Data Image to the bus buffer (firstly updates slave's output PDOs)
     * 
     * @param[in] pdi_buffer
     *    PDI buffer to write data from
     * @param[in] timeout
     *    timeout of the I/O operation
     * 
     * @throws cifx::Error
     *    on failure
     */
    inline void write_bus_impl(::ethercat::config::types::Span<const uint8_t> pdi_buffer, std::chrono::milliseconds timeout);

private: /* --------------------------------------------------- Private data ------------------------------------------------------ */

    /// Reference tot he CIFX channel used to communicate with CIFX master device
    cifx::Channel &channel;

    /// Offset of Input PDI data in the Input Cyclical I/O are of the channel
    std::size_t input_pdi_data_offset { 0 };
    /// Offset of Input PDI data in the Output Cyclical I/O are of the channel
    std::size_t output_pdi_data_offset { 0 };

};

/* ================================================================================================================================ */

} // End namespace cifx::ethercat

/* ==================================================== Implementation includes =================================================== */

#include "cifx/ethercat/master/master.hpp"

/* ================================================================================================================================ */

#endif