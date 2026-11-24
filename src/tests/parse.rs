use crate::{DataBits, FlowControl, Parity, StopBits};
use std::str::FromStr;

#[test]
fn test_databits_from_str() {
    assert_eq!(DataBits::from_str("Five"), Ok(DataBits::Five));
    assert_eq!(DataBits::from_str("five"), Ok(DataBits::Five));
    assert_eq!(DataBits::from_str("5"), Ok(DataBits::Five));

    assert_eq!(DataBits::from_str("Six"), Ok(DataBits::Six));
    assert_eq!(DataBits::from_str("six"), Ok(DataBits::Six));
    assert_eq!(DataBits::from_str("6"), Ok(DataBits::Six));

    assert_eq!(DataBits::from_str("Seven"), Ok(DataBits::Seven));
    assert_eq!(DataBits::from_str("seven"), Ok(DataBits::Seven));
    assert_eq!(DataBits::from_str("7"), Ok(DataBits::Seven));

    assert_eq!(DataBits::from_str("Eight"), Ok(DataBits::Eight));
    assert_eq!(DataBits::from_str("eight"), Ok(DataBits::Eight));
    assert_eq!(DataBits::from_str("8"), Ok(DataBits::Eight));

    assert_eq!(DataBits::from_str("Nine"), Err(()));
    assert_eq!(DataBits::from_str(""), Err(()));

    // Round-trip tests
    for &variant in &[
        DataBits::Five,
        DataBits::Six,
        DataBits::Seven,
        DataBits::Eight,
    ] {
        assert_eq!(DataBits::from_str(&variant.to_string()), Ok(variant));
        assert_eq!(DataBits::from_str(&format!("{:?}", variant)), Ok(variant));
    }
}

#[test]
fn test_parity_from_str() {
    assert_eq!(Parity::from_str("None"), Ok(Parity::None));
    assert_eq!(Parity::from_str("none"), Ok(Parity::None));
    assert_eq!(Parity::from_str("N"), Ok(Parity::None));
    assert_eq!(Parity::from_str("n"), Ok(Parity::None));

    assert_eq!(Parity::from_str("Odd"), Ok(Parity::Odd));
    assert_eq!(Parity::from_str("odd"), Ok(Parity::Odd));
    assert_eq!(Parity::from_str("O"), Ok(Parity::Odd));
    assert_eq!(Parity::from_str("o"), Ok(Parity::Odd));

    assert_eq!(Parity::from_str("Even"), Ok(Parity::Even));
    assert_eq!(Parity::from_str("even"), Ok(Parity::Even));
    assert_eq!(Parity::from_str("E"), Ok(Parity::Even));
    assert_eq!(Parity::from_str("e"), Ok(Parity::Even));

    assert_eq!(Parity::from_str("Other"), Err(()));
    assert_eq!(Parity::from_str(""), Err(()));

    // Round-trip tests
    for &variant in &[Parity::None, Parity::Odd, Parity::Even] {
        assert_eq!(Parity::from_str(&variant.to_string()), Ok(variant));
        assert_eq!(Parity::from_str(&format!("{:?}", variant)), Ok(variant));
    }
}

#[test]
fn test_stopbits_from_str() {
    assert_eq!(StopBits::from_str("One"), Ok(StopBits::One));
    assert_eq!(StopBits::from_str("one"), Ok(StopBits::One));
    assert_eq!(StopBits::from_str("1"), Ok(StopBits::One));

    assert_eq!(StopBits::from_str("Two"), Ok(StopBits::Two));
    assert_eq!(StopBits::from_str("two"), Ok(StopBits::Two));
    assert_eq!(StopBits::from_str("2"), Ok(StopBits::Two));

    assert_eq!(StopBits::from_str("Three"), Err(()));
    assert_eq!(StopBits::from_str(""), Err(()));

    // Round-trip tests
    for &variant in &[StopBits::One, StopBits::Two] {
        assert_eq!(StopBits::from_str(&variant.to_string()), Ok(variant));
        assert_eq!(StopBits::from_str(&format!("{:?}", variant)), Ok(variant));
    }
}

#[test]
fn test_flowcontrol_from_str() {
    assert_eq!(FlowControl::from_str("None"), Ok(FlowControl::None));
    assert_eq!(FlowControl::from_str("none"), Ok(FlowControl::None));
    assert_eq!(FlowControl::from_str("n"), Ok(FlowControl::None));

    assert_eq!(FlowControl::from_str("Software"), Ok(FlowControl::Software));
    assert_eq!(FlowControl::from_str("software"), Ok(FlowControl::Software));
    assert_eq!(FlowControl::from_str("SW"), Ok(FlowControl::Software));
    assert_eq!(FlowControl::from_str("sw"), Ok(FlowControl::Software));
    assert_eq!(FlowControl::from_str("s"), Ok(FlowControl::Software));

    assert_eq!(FlowControl::from_str("Hardware"), Ok(FlowControl::Hardware));
    assert_eq!(FlowControl::from_str("hardware"), Ok(FlowControl::Hardware));
    assert_eq!(FlowControl::from_str("HW"), Ok(FlowControl::Hardware));
    assert_eq!(FlowControl::from_str("hw"), Ok(FlowControl::Hardware));
    assert_eq!(FlowControl::from_str("h"), Ok(FlowControl::Hardware));

    assert_eq!(FlowControl::from_str("Other"), Err(()));
    assert_eq!(FlowControl::from_str(""), Err(()));

    // Round-trip tests
    for &variant in &[
        FlowControl::None,
        FlowControl::Software,
        FlowControl::Hardware,
    ] {
        assert_eq!(FlowControl::from_str(&variant.to_string()), Ok(variant));
        assert_eq!(FlowControl::from_str(&format!("{:?}", variant)), Ok(variant));
    }
}
