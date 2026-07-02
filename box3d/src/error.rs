#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    InvalidWorld,
    InvalidBody,
    InvalidShape,
    Null,
}

pub type Result<T> = std::result::Result<T, Error>;
