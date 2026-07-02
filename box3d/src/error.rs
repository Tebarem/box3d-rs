#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    InvalidWorld,
    InvalidBody,
    InvalidShape,
    InvalidInput,
    Null,
}

pub type Result<T> = std::result::Result<T, Error>;
