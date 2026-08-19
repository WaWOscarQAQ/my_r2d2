//! ROS interface discovery boundary.
//!
//! Paper-backed part:
//! R2D2 extracts ROS interfaces and uses their data structure to build
//! inputs. Mutation is conducted recursively based on data files from the
//! interface specification, so each field carries a recursive type tree.
//!
//! The `FileExtractor` implementation parses real `.msg` and `.srv` files;
//! ROS graph discovery is still a separate integration layer.

use std::collections::HashSet;
use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};

/// The kind of ROS interface that can carry a fuzzing payload.
///
/// Only topics and services are in scope; actions are intentionally
/// excluded per the reproduction contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Kind {
    Topic,
    Service,
}

/// ROS primitive value kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Primitive {
    Bool,
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    I64,
    U64,
    F32,
    F64,
    /// Variable-length UTF-8 string.
    String,
    /// Variable-length byte sequence (`uint8[]`).
    Bytes,
}

/// One field in a message or service shape.
#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    pub name: String,
    pub ty: TypeNode,
}

impl Field {
    pub fn new(name: impl Into<String>, ty: impl Into<TypeNode>) -> Self {
        Self {
            name: name.into(),
            ty: ty.into(),
        }
    }
}

/// A node in the recursive type tree describing an interface payload.
#[derive(Debug, Clone, PartialEq)]
pub enum TypeNode {
    /// A primitive leaf value.
    Primitive(Primitive),
    /// A nested message: an ordered sequence of named fields.
    Nested(Vec<Field>),
    /// An array over an element type. `Some(len)` fixes the length;
    /// `None` is a variable-length sequence.
    Array(Box<TypeNode>, Option<usize>),
}

impl TypeNode {
    pub fn nested(fields: Vec<Field>) -> Self {
        Self::Nested(fields)
    }

    pub fn array(element: TypeNode) -> Self {
        Self::Array(Box::new(element), None)
    }

    pub fn fixed_array(element: TypeNode, len: usize) -> Self {
        Self::Array(Box::new(element), Some(len))
    }
}

/// Wraps a primitive as a leaf type node. Lossless and infallible, so this
/// is a genuine `From`; `Into<TypeNode>` for `Primitive` comes for free.
impl From<Primitive> for TypeNode {
    fn from(primitive: Primitive) -> Self {
        Self::Primitive(primitive)
    }
}

/// A data file associated with an interface.
///
/// The parsed type tree lives in `Field::ty`; this structure records the
/// file name and raw source for provenance and the reproduction report.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DataFile {
    /// File path or message type name, e.g. `geometry_msgs/msg/Twist.msg`.
    pub name: String,
    /// Raw `.msg` or `.srv` source text, unparsed in this placeholder.
    pub source: String,
}

impl DataFile {
    pub fn new(name: impl Into<String>, source: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            source: source.into(),
        }
    }
}

/// A discovered ROS interface and its top-level fields.
#[derive(Debug, Clone, PartialEq)]
pub struct Interface {
    pub name: String,
    pub kind: Kind,
    pub fields: Vec<Field>,
    pub data_files: Vec<DataFile>,
    /// For services, the request and response message shapes. `fields` keeps
    /// the request fields for compatibility with the original payload model.
    pub service: Option<Service>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Service {
    pub request: Vec<Field>,
    pub response: Vec<Field>,
}

impl Interface {
    pub fn new(name: impl Into<String>, kind: Kind, fields: Vec<Field>) -> Self {
        Self {
            name: name.into(),
            kind,
            fields,
            data_files: Vec::new(),
            service: None,
        }
    }

    pub fn with_data_files(mut self, data_files: Vec<DataFile>) -> Self {
        self.data_files = data_files;
        self
    }

    pub fn with_service(mut self, request: Vec<Field>, response: Vec<Field>) -> Self {
        self.service = Some(Service { request, response });
        self
    }
}

/// Error returned when interface discovery fails.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Error {
    msg: String,
}

impl Error {
    pub fn new(msg: impl Into<String>) -> Self {
        Self { msg: msg.into() }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.msg)
    }
}

impl std::error::Error for Error {}

/// Abstracts interface discovery from ROS 2.
pub trait Extractor {
    fn extract(&self) -> Result<Vec<Interface>, Error>;
}

/// Extracts ROS 2 interfaces directly from real `.msg` and `.srv` files.
///
/// `search_paths` should contain directories that contain ROS packages, for
/// example an install/share directory. The files passed to `new` are the
/// top-level interfaces to extract; referenced message files are resolved
/// from the current package and then from `search_paths`.
pub struct FileExtractor {
    files: Vec<PathBuf>,
    search_paths: Vec<PathBuf>,
}

impl FileExtractor {
    pub fn new(files: Vec<impl Into<PathBuf>>, search_paths: Vec<impl Into<PathBuf>>) -> Self {
        Self {
            files: files.into_iter().map(Into::into).collect(),
            search_paths: search_paths.into_iter().map(Into::into).collect(),
        }
    }

    pub fn from_file(path: impl Into<PathBuf>) -> Self {
        Self::new(vec![path.into()], Vec::<PathBuf>::new())
    }
}

impl Extractor for FileExtractor {
    fn extract(&self) -> Result<Vec<Interface>, Error> {
        self.files
            .iter()
            .map(|path| {
                Parser {
                    search_paths: &self.search_paths,
                    loaded: HashSet::new(),
                }
                .parse_interface(path)
            })
            .collect()
    }
}

struct Parser<'a> {
    search_paths: &'a [PathBuf],
    loaded: HashSet<PathBuf>,
}

impl Parser<'_> {
    fn parse_interface(&mut self, path: &Path) -> Result<Interface, Error> {
        
        let path = canonical(path)?;

        let raw = fs::read_to_string(&path);

        let source = match raw {
            Ok(content) => content,
            Err(err) => {
                let msg = format!("read {}: {err}", path.display());
                return Err(Error::new(msg));
            }
        };

        let package = package_name(&path)?;
        
        let stem = path.file_stem();
        let stem_str = stem.and_then(|s| s.to_str());
        
        let name = match stem_str {
            Some(name) => name.to_string(),
            None => {
                let msg = format!("invalid interface filename: {}", path.display());
                return Err(Error::new(msg));
            }
        };

        let mut data_files = Vec::new();

        let extension = path.extension().and_then(|s| s.to_str());

        let (request, response) = match extension {

            Some("srv") => {
                let sections = split_service(&source)?;
                (
                    self.parse_fields(&sections.0, &path, &package, &mut data_files)?,
                    self.parse_fields(&sections.1, &path, &package, &mut data_files)?,
                )
            }

            Some("msg") => (
                self.parse_fields(&source, &path, &package, &mut data_files)?,
                Vec::new(),
            ),

            _ => {
                let msg = format!("{} is not a .msg or .srv file", path.display());
                return Err(Error::new(msg));
            }

        };
        
        let is_service = extension == Some("srv");

        let name_str = path.display().to_string();
        let self_file = DataFile::new(name_str, source);
        data_files.insert(0, self_file);

        let kind = if is_service { Kind::Service } else { Kind::Topic };

        let request_fields = request.clone();
        let mut interface = Interface::new(name, kind, request_fields);
        interface = interface.with_data_files(data_files);

        if is_service {
            interface = interface.with_service(request, response);
        }
        Ok(interface)

    }

    fn parse_fields(
        &mut self,
        source: &str,
        owner: &Path,
        package: &str,
        data_files: &mut Vec<DataFile>,
    ) -> Result<Vec<Field>, Error> {

        let mut fields = Vec::new();

        for (line_no, raw) in source.lines().enumerate() {

            let line = raw.split('#').next().unwrap_or("").trim();

            if line.is_empty() { continue; }

            if line.contains('=') {
                let error_msg = format!( "{}:{} constants are not represented ", owner.display(), line_no + 1);
                return Err(Error::new(error_msg));
            }

            let mut parts = line.split_whitespace();
            let type_token = parts
                .next()
                .ok_or_else(|| Error::new("missing field type"))?;

            let field_name = parts.next().ok_or_else(|| {
                let msg = format!("{}:{} missing field name", owner.display(), line_no + 1);
                Error::new(msg)
            })?;

            if parts.next().is_some() {
                let msg = format!("{}:{} defaults are not represented by the current model", 
                                  owner.display(), line_no + 1);
                return Err(Error::new(msg));
            }
            fields.push(Field::new(
                field_name,
                self.parse_type(type_token, owner, package, data_files)?,
            ));
        }
        Ok(fields)
    }

    fn parse_type(
        &mut self,
        token: &str,
        owner: &Path,
        package: &str,
        data_files: &mut Vec<DataFile>,
    ) -> Result<TypeNode, Error> {
        let (base, array) = split_array(token)?;
        if base.contains("<=") {
            return Err(Error::new(format!(
                "bounded type {token:?} is not represented by the current TypeNode model"
            )));
        }
        let node = match primitive(base) {
            Some(p) => TypeNode::Primitive(p),
            None => {
                let nested = self.resolve_nested(base, owner, package)?;
                let raw = fs::read_to_string(&nested);
                let source = match raw {
                    Ok(content) => content,
                    Err(err) => {
                        let msg = format!("read {}: {err}", nested.display());
                        return Err(Error::new(msg));
                    }
                };
                if self.loaded.insert(nested.clone()) {
                    data_files.push(DataFile::new(nested.display().to_string(), source.clone()));
                }
                let nested_package = package_name(&nested)?;
                TypeNode::Nested(self.parse_fields(
                    &source,
                    &nested,
                    &nested_package,
                    data_files,
                )?)
            }
        };
        Ok(match array {
            None => node,
            Some(Some(length)) => TypeNode::fixed_array(node, length),
            Some(None) => TypeNode::array(node),
        })
    }

    fn resolve_nested(&self, token: &str, owner: &Path, package: &str) -> Result<PathBuf, Error> {
        let (pkg, name) = token
            .split_once('/')
            .map_or((package, token), |(p, n)| (p, n));
        let name = name.strip_prefix("msg/").unwrap_or(name);
        let mut candidates = vec![
            owner
                .parent()
                .unwrap_or(Path::new("."))
                .join(format!("{name}.msg")),
        ];
        for root in self.search_paths {
            candidates.push(root.join(pkg).join("msg").join(format!("{name}.msg")));
        }
        candidates
            .into_iter()
            .find(|candidate| candidate.is_file())
            .ok_or_else(|| Error::new(format!("cannot resolve nested ROS message type {token:?}")))
    }
}

fn canonical(path: &Path) -> Result<PathBuf, Error> {
    match fs::canonicalize(path) {
        Ok(resolved) => Ok(resolved),
        Err(err) => {
            let msg = format!("canonicalize {}: {err}", path.display());
            Err(Error::new(msg))
        }
    }
}

fn package_name(path: &Path) -> Result<String, Error> {
    path.parent()
        .and_then(Path::parent)
        .and_then(Path::file_name)
        .and_then(|s| s.to_str())
        .map(str::to_string)
        .ok_or_else(|| Error::new(format!("cannot infer ROS package for {}", path.display())))
}

fn split_service(source: &str) -> Result<(String, String), Error> {
    let mut sections = source.split("\n---\n");
    let request = sections.next().unwrap_or_default().to_string();
    let response = sections
        .next()
        .ok_or_else(|| Error::new("service file must contain a line containing only ---"))?
        .to_string();
    if sections.next().is_some() {
        return Err(Error::new(
            "service file contains more than one --- separator",
        ));
    }
    Ok((request, response))
}

fn split_array(token: &str) -> Result<(&str, Option<Option<usize>>), Error> {
    let Some(open) = token.find('[') else {
        return Ok((token, None));
    };
    if !token.ends_with(']') {
        return Err(Error::new(format!("malformed array type {token:?}")));
    }
    let base = &token[..open];
    let suffix = &token[open + 1..token.len() - 1];
    if suffix.is_empty() {
        Ok((base, Some(None)))
    } else {
        let length = suffix
            .parse::<usize>()
            .map_err(|_| Error::new(format!("unsupported array bound in {token:?}")))?;
        Ok((base, Some(Some(length))))
    }
}

fn primitive(token: &str) -> Option<Primitive> {
    Some(match token {
        "bool" => Primitive::Bool,
        "int8" => Primitive::I8,
        "uint8" | "byte" | "char" => Primitive::U8,
        "int16" => Primitive::I16,
        "uint16" => Primitive::U16,
        "int32" => Primitive::I32,
        "uint32" => Primitive::U32,
        "int64" => Primitive::I64,
        "uint64" => Primitive::U64,
        "float32" => Primitive::F32,
        "float64" => Primitive::F64,
        "string" => Primitive::String,
        _ => return None,
    })
}
