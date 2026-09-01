//! Message types hand-written to the **Humble** field set, where the bundled
//! `hiroz-msgs` definitions do not match what this stack is on the wire with.
//!
//! # The defect
//!
//! `hiroz-msgs` 0.2.0 ships **Jazzy** `.msg` definitions and generates its
//! types from those. The `humble` cargo feature only changes *type-hash*
//! handling (`humble` implies `no-type-hash`); it does not select a different
//! set of message definitions. Any interface whose fields changed between
//! Humble and Jazzy is therefore generated with the wrong layout, and CDR is a
//! positional encoding with no field names to save you.
//!
//! In this workspace exactly one type is affected: `sensor_msgs/Range` gained
//! a trailing `float32 variance` in Jazzy. `rosflight_sim` runs on Humble and
//! publishes the six-field message, so the bundled seven-field type runs off
//! the end of every payload:
//!
//! ```text
//! [WARN] … cannot decode a range message: CDR deserialization error: unexpected end of input
//! ```
//!
//! at the sensor's 20 Hz — which is the range finder silently never reaching
//! the firmware.
//!
//! Everything else this node touches was checked against
//! `/opt/ros/humble/share` and is wire-identical: `sensor_msgs/Imu`,
//! `Temperature`, `MagneticField`, `std_msgs/Header`, `builtin_interfaces/Time`,
//! `geometry_msgs/Vector3` and `std_srvs/Trigger`. The `rosflight_msgs` types
//! are generated from this repo's own vendored definitions
//! (`sim/msg_vendor/rosflight_msgs`) and are unaffected by any of this.
//!
//! # The fix
//!
//! [`RangeCompat`] below: the Humble field set, reusing the bundled `Range`'s
//! type name and hash so the wire identity — and therefore the zenoh key
//! expression the subscription matches on — is unchanged. Only the payload
//! layout differs.
//!
//! This is the same pattern the interop spike established for
//! `TriggerRequestCompat` in `rosplane_rs/rosplane_nodes/src/shim/compat_msgs.rs`
//! (a different defect: empty-message padding), applied to a different cause.
//!
//! It is also safe in the other direction. If this stack ever moves to a Jazzy
//! publisher, `RangeCompat` will simply leave the trailing `variance` bytes
//! unread, which the deserializer tolerates — the failure only ever runs one
//! way, off the end of a short buffer. Delete this module when `hiroz-msgs`
//! ships per-distro definitions.

use hiroz::msg::{SerdeCdrSerdes, ZMessage};
use hiroz::{MessageTypeInfo, entity};
use veloxity_ros_msgs::sensor_msgs::Range;
use veloxity_ros_msgs::std_msgs::Header;

/// `sensor_msgs/msg/Range` as **Humble** defines it: no `variance`.
// No `PartialEq`: the bundled `std_msgs::Header` does not derive it, so
// neither can this. The tests compare field by field.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct RangeCompat {
    pub header: Header,
    pub radiation_type: u8,
    pub field_of_view: f32,
    pub min_range: f32,
    pub max_range: f32,
    pub range: f32,
}

impl MessageTypeInfo for RangeCompat {
    fn type_name() -> &'static str {
        // The same name the generated type reports: this must stay identical
        // or the key expression changes and nothing matches.
        "sensor_msgs::msg::dds_::Range_"
    }

    fn type_hash() -> entity::TypeHash {
        <Range as MessageTypeInfo>::type_hash()
    }
}

impl hiroz::ros_msg::WithTypeInfo for RangeCompat {}

impl ZMessage for RangeCompat {
    type Serdes = SerdeCdrSerdes<Self>;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> RangeCompat {
        RangeCompat {
            header: Header {
                stamp: veloxity_ros_msgs::builtin_interfaces::Time { sec: 7, nanosec: 8 },
                frame_id: String::new(),
            },
            radiation_type: 1, // Range::INFRARED
            field_of_view: 0.5,
            min_range: 0.25,
            max_range: 25.0,
            range: 3.5,
        }
    }

    /// A Humble `Range` payload built by hand, little-endian XCDR1. The
    /// offsets in the comments are relative to the start of the payload, i.e.
    /// after the four-byte encapsulation header, which is what CDR aligns to.
    const HUMBLE_RANGE_BYTES: &[u8] = &[
        0x00, 0x01, 0x00, 0x00, // encapsulation: CDR_LE, options 0
        0x07, 0x00, 0x00, 0x00, //  0: header.stamp.sec     = 7
        0x08, 0x00, 0x00, 0x00, //  4: header.stamp.nanosec = 8
        0x01, 0x00, 0x00, 0x00, //  8: header.frame_id length = 1 (the NUL)
        0x00, //                   12: header.frame_id = ""
        0x01, //                   13: radiation_type = INFRARED
        0x00, 0x00, //             14: padding, realigning f32 to 4
        0x00, 0x00, 0x00, 0x3f, // 16: field_of_view = 0.5
        0x00, 0x00, 0x80, 0x3e, // 20: min_range     = 0.25
        0x00, 0x00, 0xc8, 0x41, // 24: max_range     = 25.0
        0x00, 0x00, 0x60, 0x40, // 28: range         = 3.5
    ];

    /// The whole point: the Humble payload is exactly four bytes — one
    /// `float32 variance` — shorter than the bundled Jazzy one. If this ever
    /// comes out equal, `hiroz-msgs` has started shipping Humble definitions
    /// and this module can go.
    #[test]
    fn the_humble_range_is_four_bytes_shorter_than_the_bundled_one() {
        let compat = sample().serialize();
        let bundled = Range {
            header: sample().header,
            radiation_type: 1,
            field_of_view: 0.5,
            min_range: 0.25,
            max_range: 25.0,
            range: 3.5,
            variance: 0.0,
        }
        .serialize();

        assert_eq!(
            compat.len() + 4,
            bundled.len(),
            "the difference is exactly the trailing float32 variance"
        );
        // …and everything up to that field is byte-identical, which is what
        // makes reusing the wire identity legitimate.
        assert_eq!(compat[..], bundled[..compat.len()]);
    }

    /// The regression this module exists for: the bundled type cannot read a
    /// Humble payload, and `RangeCompat` can.
    #[test]
    fn a_humble_payload_decodes_only_with_the_compat_type() {
        assert!(
            Range::deserialize(HUMBLE_RANGE_BYTES).is_err(),
            "the bundled Jazzy type must still run off the end — if it does \
             not, the bundled definitions changed and this module can go"
        );

        let decoded = RangeCompat::deserialize(HUMBLE_RANGE_BYTES).expect("must deserialize");

        assert_eq!(decoded.header.stamp.sec, 7);
        assert_eq!(decoded.header.stamp.nanosec, 8);
        assert_eq!(decoded.header.frame_id, "");
        assert_eq!(decoded.radiation_type, 1);
        assert_eq!(decoded.field_of_view, 0.5);
        assert_eq!(decoded.min_range, 0.25);
        assert_eq!(decoded.max_range, 25.0);
        assert_eq!(decoded.range, 3.5);
    }

    /// The hand-built buffer is what this type actually writes, so the layout
    /// above is a pin on the encoding rather than a guess about it.
    #[test]
    fn the_compat_type_round_trips_through_that_exact_layout() {
        assert_eq!(sample().serialize(), HUMBLE_RANGE_BYTES);

        let back = RangeCompat::deserialize(&sample().serialize()).unwrap();
        assert_eq!(back.serialize(), HUMBLE_RANGE_BYTES);
    }

    /// Reusing the generated type's identity is what keeps the key expression
    /// — and therefore the match against the `rosflight_sim` publisher —
    /// unchanged.
    #[test]
    fn the_wire_identity_is_the_bundled_range() {
        assert_eq!(
            <RangeCompat as MessageTypeInfo>::type_name(),
            <Range as MessageTypeInfo>::type_name()
        );
        assert_eq!(
            <RangeCompat as MessageTypeInfo>::type_hash(),
            <Range as MessageTypeInfo>::type_hash()
        );
    }
}
