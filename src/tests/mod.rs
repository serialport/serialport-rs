use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(test)] {
        pub(crate) mod timeout;
    }
}
