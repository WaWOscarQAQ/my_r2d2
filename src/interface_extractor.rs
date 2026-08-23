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
/// Topics, services, and parameter writes are in scope; actions are
/// intentionally excluded per the reproduction contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Kind {
    Topic,
    Service,
    Parameter,
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
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Field {
    pub name: String,
    pub ty: TypeNode,
    pub default_value: Option<Literal>,
}

impl Field {
    pub fn new(name: impl Into<String>, ty: impl Into<TypeNode>) -> Self {
        Self {
            name: name.into(),
            ty: ty.into(),
            default_value: None,
        }
    }

    pub fn with_default(mut self, default_value: Literal) -> Self {
        self.default_value = Some(default_value);
        self
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Literal {
    Bool(bool),
    I8(i8),
    U8(u8),
    I16(i16),
    U16(u16),
    I32(i32),
    U32(u32),
    I64(i64),
    U64(u64),
    F32(u32),
    F64(u64),
    String(String),
    Nested(Vec<Literal>),
    Array(Vec<Literal>),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Constant {
    pub name: String,
    pub ty: TypeNode,
    pub value: Literal,
}

/// A node in the recursive type tree describing an interface payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TypeNode {
    /// A primitive leaf value.
    Primitive(Primitive),
    /// A nested message: an ordered sequence of named fields.
    Nested(Vec<Field>),
    /// An array over an element type. `Some(len)` fixes the length;
    /// `None` is a variable-length sequence.
    Array(Box<TypeNode>, Option<usize>),
    /// Extra constraints over an underlying node, such as bounded strings or
    /// bounded sequences.
    Constrained(Box<TypeNode>, Constraint),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Constraint {
    StringMaxLen(usize),
    ArrayMaxLen(usize),
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

    pub fn bounded_string(len: usize) -> Self {
        Self::Constrained(
            Box::new(Self::Primitive(Primitive::String)),
            Constraint::StringMaxLen(len),
        )
    }

    pub fn bounded_array(element: TypeNode, len: usize) -> Self {
        Self::Constrained(Box::new(Self::array(element)), Constraint::ArrayMaxLen(len))
    }

    pub fn as_primitive(&self) -> Option<Primitive> {
        match self {
            Self::Primitive(primitive) => Some(*primitive),
            Self::Constrained(inner, _) => inner.as_primitive(),
            _ => None,
        }
    }

    pub fn as_nested(&self) -> Option<&[Field]> {
        match self {
            Self::Nested(fields) => Some(fields),
            Self::Constrained(inner, _) => inner.as_nested(),
            _ => None,
        }
    }

    pub fn as_array(&self) -> Option<(&TypeNode, Option<usize>)> {
        match self {
            Self::Array(element, fixed_len) => Some((element.as_ref(), *fixed_len)),
            Self::Constrained(inner, _) => inner.as_array(),
            _ => None,
        }
    }

    pub fn string_bound(&self) -> Option<usize> {
        match self {
            Self::Constrained(inner, Constraint::StringMaxLen(max))
                if inner.as_primitive() == Some(Primitive::String) =>
            {
                Some(*max)
            }
            Self::Constrained(inner, _) => inner.string_bound(),
            _ => None,
        }
    }

    pub fn array_bound(&self) -> Option<usize> {
        match self {
            Self::Constrained(inner, Constraint::ArrayMaxLen(max))
                if inner.as_array().is_some() =>
            {
                Some(*max)
            }
            Self::Constrained(inner, _) => inner.array_bound(),
            _ => None,
        }
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
    /// Parsed constants declared in this file.
    pub constants: Vec<Constant>,
}

impl DataFile {
    pub fn new(name: impl Into<String>, source: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            source: source.into(),
            constants: Vec::new(),
        }
    }

    pub fn with_constants(mut self, constants: Vec<Constant>) -> Self {
        self.constants = constants;
        self
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

struct ParsedMembers {
    fields: Vec<Field>,
    constants: Vec<Constant>,
}

enum ParsedArray {
    None,
    Unbounded,
    Fixed(usize),
    Bounded(usize),
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

        let (request, response, constants) = match extension {
            Some("srv") => {
                let sections = split_service(&source)?;
                let request_members =
                    self.parse_members(&sections.0, &path, &package, &mut data_files)?;
                let response_members =
                    self.parse_members(&sections.1, &path, &package, &mut data_files)?;
                (
                    request_members.fields,
                    response_members.fields,
                    request_members
                        .constants
                        .into_iter()
                        .chain(response_members.constants)
                        .collect(),
                )
            }

            Some("msg") => {
                let members = self.parse_members(&source, &path, &package, &mut data_files)?;
                (members.fields, Vec::new(), members.constants)
            }

            _ => {
                let msg = format!("{} is not a .msg or .srv file", path.display());
                return Err(Error::new(msg));
            }
        };

        let is_service = extension == Some("srv");

        let name_str = path.display().to_string();
        let self_file = DataFile::new(name_str, source).with_constants(constants);
        data_files.insert(0, self_file);

        let kind = if is_service {
            Kind::Service
        } else {
            Kind::Topic
        };

        let request_fields = request.clone();
        let mut interface = Interface::new(name, kind, request_fields);
        interface = interface.with_data_files(data_files);

        if is_service {
            interface = interface.with_service(request, response);
        }
        Ok(interface)
    }

    fn parse_members(
        &mut self,
        source: &str,
        owner: &Path,
        package: &str,
        data_files: &mut Vec<DataFile>,
    ) -> Result<ParsedMembers, Error> {
        let mut fields = Vec::new();
        let mut constants = Vec::new();

        for (line_no, raw) in source.lines().enumerate() {
            let line = raw.split('#').next().unwrap_or("").trim();

            if line.is_empty() {
                continue;
            }

            let parsed = parse_member_line(line, owner, line_no + 1)?;
            let ty = self.parse_type(parsed.type_token, owner, package, data_files)?;
            if parsed.is_constant {
                let value = parse_literal(
                    parsed.value.ok_or_else(|| {
                        Error::new(format!(
                            "{}:{} missing constant value",
                            owner.display(),
                            line_no + 1
                        ))
                    })?,
                    &ty,
                    owner,
                    line_no + 1,
                )?;
                constants.push(Constant {
                    name: parsed.name.to_string(),
                    ty,
                    value,
                });
                continue;
            }

            let mut field = Field::new(parsed.name, ty);
            if let Some(value) = parsed.value {
                let default_value = parse_literal(value, &field.ty, owner, line_no + 1)?;
                field = field.with_default(default_value);
            }
            fields.push(field);
        }
        Ok(ParsedMembers { fields, constants })
    }

    fn parse_type(
        &mut self,
        token: &str,
        owner: &Path,
        package: &str,
        data_files: &mut Vec<DataFile>,
    ) -> Result<TypeNode, Error> {
        let (base, array) = split_array(token)?;
        let node = match primitive_type(base) {
            Some(p) => TypeNode::Primitive(p),
            None if base == "wstring" => TypeNode::Primitive(Primitive::String),
            None if base.starts_with("string<=") => {
                let len = parse_bound_suffix(base, "string")?;
                TypeNode::bounded_string(len)
            }
            None if base.starts_with("wstring<=") => {
                let len = parse_bound_suffix(base, "wstring")?;
                TypeNode::bounded_string(len)
            }
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
                let nested_package = package_name(&nested)?;
                let nested_members =
                    self.parse_members(&source, &nested, &nested_package, data_files)?;
                if self.loaded.insert(nested.clone()) {
                    data_files.push(
                        DataFile::new(nested.display().to_string(), source.clone())
                            .with_constants(nested_members.constants.clone()),
                    );
                }
                TypeNode::Nested(nested_members.fields)
            }
        };
        Ok(match array {
            ParsedArray::None => node,
            ParsedArray::Fixed(length) => TypeNode::fixed_array(node, length),
            ParsedArray::Unbounded => TypeNode::array(node),
            ParsedArray::Bounded(length) => TypeNode::bounded_array(node, length),
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

struct ParsedMemberLine<'a> {
    type_token: &'a str,
    name: &'a str,
    value: Option<&'a str>,
    is_constant: bool,
}

fn parse_member_line<'a>(
    line: &'a str,
    owner: &Path,
    line_no: usize,
) -> Result<ParsedMemberLine<'a>, Error> {
    let mut pieces = line.splitn(2, char::is_whitespace);
    let type_token = pieces
        .next()
        .filter(|token| !token.is_empty())
        .ok_or_else(|| Error::new("missing field type"))?;
    let rest = pieces.next().unwrap_or("").trim_start();
    if rest.is_empty() {
        return Err(Error::new(format!(
            "{}:{} missing field name",
            owner.display(),
            line_no
        )));
    }
    let name_end = rest
        .find(|ch: char| ch.is_whitespace() || ch == '=')
        .unwrap_or(rest.len());
    let name = &rest[..name_end];
    if name.is_empty() {
        return Err(Error::new(format!(
            "{}:{} missing field name",
            owner.display(),
            line_no
        )));
    }
    let tail = rest[name_end..].trim_start();
    if let Some(value) = tail.strip_prefix('=') {
        return Ok(ParsedMemberLine {
            type_token,
            name,
            value: Some(value.trim_start()),
            is_constant: true,
        });
    }
    Ok(ParsedMemberLine {
        type_token,
        name,
        value: (!tail.is_empty()).then_some(tail),
        is_constant: false,
    })
}

fn split_array(token: &str) -> Result<(&str, ParsedArray), Error> {
    let Some(open) = token.find('[') else {
        return Ok((token, ParsedArray::None));
    };
    if !token.ends_with(']') {
        return Err(Error::new(format!("malformed array type {token:?}")));
    }
    let base = &token[..open];
    let suffix = &token[open + 1..token.len() - 1];
    if suffix.is_empty() {
        Ok((base, ParsedArray::Unbounded))
    } else if let Some(bound) = suffix.strip_prefix("<=") {
        let length = bound
            .parse::<usize>()
            .map_err(|_| Error::new(format!("unsupported array bound in {token:?}")))?;
        Ok((base, ParsedArray::Bounded(length)))
    } else {
        let length = suffix
            .parse::<usize>()
            .map_err(|_| Error::new(format!("unsupported array bound in {token:?}")))?;
        Ok((base, ParsedArray::Fixed(length)))
    }
}

fn primitive_type(token: &str) -> Option<Primitive> {
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

fn parse_bound_suffix(token: &str, prefix: &str) -> Result<usize, Error> {
    token
        .strip_prefix(prefix)
        .and_then(|suffix| suffix.strip_prefix("<="))
        .ok_or_else(|| Error::new(format!("unsupported bounded type {token:?}")))?
        .parse::<usize>()
        .map_err(|_| Error::new(format!("unsupported bounded type {token:?}")))
}

fn parse_literal(raw: &str, ty: &TypeNode, owner: &Path, line_no: usize) -> Result<Literal, Error> {
    match ty {
        TypeNode::Primitive(primitive) => parse_scalar_literal(raw, *primitive, owner, line_no),
        TypeNode::Array(element, fixed_len) => {
            let items = parse_array_literal(raw, owner, line_no)?;
            if let Some(expected) = fixed_len
                && items.len() != *expected
            {
                return Err(Error::new(format!(
                    "{}:{} fixed array default length mismatch: expected {}, found {}",
                    owner.display(),
                    line_no,
                    expected,
                    items.len()
                )));
            }
            let mut values = Vec::with_capacity(items.len());
            for item in items {
                values.push(parse_literal(item, element, owner, line_no)?);
            }
            Ok(Literal::Array(values))
        }
        TypeNode::Constrained(inner, constraint) => {
            let literal = parse_literal(raw, inner, owner, line_no)?;
            validate_literal_constraint(&literal, constraint, owner, line_no)?;
            Ok(literal)
        }
        TypeNode::Nested(fields) => parse_nested_literal(raw, fields, owner, line_no),
    }
}

fn parse_nested_literal(
    raw: &str,
    fields: &[Field],
    owner: &Path,
    line_no: usize,
) -> Result<Literal, Error> {
    let trimmed = raw.trim();
    if trimmed == "{}" {
        let values = fields
            .iter()
            .map(|field| {
                field.default_value.clone().ok_or_else(|| {
                    Error::new(format!(
                        "{}:{} nested default missing field {:?}",
                        owner.display(),
                        line_no,
                        field.name
                    ))
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        return Ok(Literal::Nested(values));
    }
    if !trimmed.starts_with('{') || !trimmed.ends_with('}') {
        return Err(Error::new(format!(
            "{}:{} nested default must be braced: {raw:?}",
            owner.display(),
            line_no
        )));
    }

    let inner = &trimmed[1..trimmed.len() - 1];
    let entries = split_top_level_items(inner, owner, line_no, "nested default")?;
    let mut keyed = Vec::with_capacity(entries.len());
    for entry in entries {
        if entry.is_empty() {
            continue;
        }
        let (name, value) = split_named_literal(entry, owner, line_no)?;
        if keyed.iter().any(|(existing, _)| *existing == name) {
            return Err(Error::new(format!(
                "{}:{} duplicate nested field {:?}",
                owner.display(),
                line_no,
                name
            )));
        }
        keyed.push((name, value));
    }

    let mut values = Vec::with_capacity(fields.len());
    for field in fields {
        let parsed = keyed
            .iter()
            .find(|(name, _)| *name == field.name)
            .map(|(_, value)| parse_literal(value, &field.ty, owner, line_no))
            .transpose()?;
        let value = match parsed.or_else(|| field.default_value.clone()) {
            Some(value) => value,
            None => {
                return Err(Error::new(format!(
                    "{}:{} nested default missing field {:?}",
                    owner.display(),
                    line_no,
                    field.name
                )));
            }
        };
        values.push(value);
    }

    for (name, _) in keyed {
        if !fields.iter().any(|field| field.name == name) {
            return Err(Error::new(format!(
                "{}:{} unknown nested field {:?}",
                owner.display(),
                line_no,
                name
            )));
        }
    }
    Ok(Literal::Nested(values))
}

fn parse_scalar_literal(
    raw: &str,
    primitive: Primitive,
    owner: &Path,
    line_no: usize,
) -> Result<Literal, Error> {
    let parse_err = |detail: &str| {
        Error::new(format!(
            "{}:{} cannot parse {raw:?} as {detail}",
            owner.display(),
            line_no
        ))
    };
    Ok(match primitive {
        Primitive::Bool => Literal::Bool(match raw {
            "true" => true,
            "false" => false,
            _ => return Err(parse_err("bool")),
        }),
        Primitive::I8 => Literal::I8(raw.parse::<i8>().map_err(|_| parse_err("int8"))?),
        Primitive::U8 => Literal::U8(raw.parse::<u8>().map_err(|_| parse_err("uint8"))?),
        Primitive::I16 => Literal::I16(raw.parse::<i16>().map_err(|_| parse_err("int16"))?),
        Primitive::U16 => Literal::U16(raw.parse::<u16>().map_err(|_| parse_err("uint16"))?),
        Primitive::I32 => Literal::I32(raw.parse::<i32>().map_err(|_| parse_err("int32"))?),
        Primitive::U32 => Literal::U32(raw.parse::<u32>().map_err(|_| parse_err("uint32"))?),
        Primitive::I64 => Literal::I64(raw.parse::<i64>().map_err(|_| parse_err("int64"))?),
        Primitive::U64 => Literal::U64(raw.parse::<u64>().map_err(|_| parse_err("uint64"))?),
        Primitive::F32 => Literal::F32(
            raw.parse::<f32>()
                .map_err(|_| parse_err("float32"))?
                .to_bits(),
        ),
        Primitive::F64 => Literal::F64(
            raw.parse::<f64>()
                .map_err(|_| parse_err("float64"))?
                .to_bits(),
        ),
        Primitive::String => Literal::String(parse_string_literal(raw, owner, line_no)?),
        Primitive::Bytes => {
            return Err(parse_err("byte sequence"));
        }
    })
}

fn parse_string_literal(raw: &str, owner: &Path, line_no: usize) -> Result<String, Error> {
    let mut chars = raw.chars();
    let quote = chars.next().ok_or_else(|| {
        Error::new(format!(
            "{}:{} empty string literal",
            owner.display(),
            line_no
        ))
    })?;
    if quote != '"' && quote != '\'' {
        return Err(Error::new(format!(
            "{}:{} string literal must be quoted: {raw:?}",
            owner.display(),
            line_no
        )));
    }
    if !raw.ends_with(quote) || raw.len() < 2 {
        return Err(Error::new(format!(
            "{}:{} unterminated string literal: {raw:?}",
            owner.display(),
            line_no
        )));
    }
    let inner = &raw[1..raw.len() - 1];
    let mut out = String::with_capacity(inner.len());
    let mut escaped = false;
    for ch in inner.chars() {
        if escaped {
            out.push(match ch {
                'n' => '\n',
                'r' => '\r',
                't' => '\t',
                '\\' => '\\',
                '\'' => '\'',
                '"' => '"',
                other => other,
            });
            escaped = false;
        } else if ch == '\\' {
            escaped = true;
        } else {
            out.push(ch);
        }
    }
    if escaped {
        return Err(Error::new(format!(
            "{}:{} dangling escape in string literal: {raw:?}",
            owner.display(),
            line_no
        )));
    }
    Ok(out)
}

fn parse_array_literal<'a>(
    raw: &'a str,
    owner: &Path,
    line_no: usize,
) -> Result<Vec<&'a str>, Error> {
    let trimmed = raw.trim();
    if trimmed == "[]" {
        return Ok(Vec::new());
    }
    if !trimmed.starts_with('[') || !trimmed.ends_with(']') {
        return Err(Error::new(format!(
            "{}:{} array default must be bracketed: {raw:?}",
            owner.display(),
            line_no
        )));
    }
    split_top_level_items(
        &trimmed[1..trimmed.len() - 1],
        owner,
        line_no,
        "array default",
    )
}

fn split_top_level_items<'a>(
    inner: &'a str,
    owner: &Path,
    line_no: usize,
    context: &str,
) -> Result<Vec<&'a str>, Error> {
    let mut out = Vec::new();
    let mut start = 0usize;
    let mut quote: Option<char> = None;
    let mut escaped = false;
    let mut brace_depth = 0usize;
    let mut bracket_depth = 0usize;
    for (index, ch) in inner.char_indices() {
        if let Some(active) = quote {
            if escaped {
                escaped = false;
                continue;
            }
            if ch == '\\' {
                escaped = true;
            } else if ch == active {
                quote = None;
            }
            continue;
        }
        match ch {
            '\'' | '"' => quote = Some(ch),
            '{' => brace_depth += 1,
            '}' => {
                if brace_depth == 0 {
                    return Err(Error::new(format!(
                        "{}:{} unmatched }} in {}",
                        owner.display(),
                        line_no,
                        context
                    )));
                }
                brace_depth -= 1;
            }
            '[' => bracket_depth += 1,
            ']' => {
                if bracket_depth == 0 {
                    return Err(Error::new(format!(
                        "{}:{} unmatched ] in {}",
                        owner.display(),
                        line_no,
                        context
                    )));
                }
                bracket_depth -= 1;
            }
            ',' if brace_depth == 0 && bracket_depth == 0 => {
                out.push(inner[start..index].trim());
                start = index + 1;
            }
            _ => {}
        }
    }
    if quote.is_some() {
        return Err(Error::new(format!(
            "{}:{} unterminated string in {}",
            owner.display(),
            line_no,
            context
        )));
    }
    if brace_depth != 0 || bracket_depth != 0 {
        return Err(Error::new(format!(
            "{}:{} unbalanced nested structure in {}",
            owner.display(),
            line_no,
            context
        )));
    }
    let tail = inner[start..].trim();
    if !tail.is_empty() {
        out.push(tail);
    }
    Ok(out)
}

fn split_named_literal<'a>(
    entry: &'a str,
    owner: &Path,
    line_no: usize,
) -> Result<(&'a str, &'a str), Error> {
    let mut quote: Option<char> = None;
    let mut escaped = false;
    let mut brace_depth = 0usize;
    let mut bracket_depth = 0usize;
    for (index, ch) in entry.char_indices() {
        if let Some(active) = quote {
            if escaped {
                escaped = false;
                continue;
            }
            if ch == '\\' {
                escaped = true;
            } else if ch == active {
                quote = None;
            }
            continue;
        }
        match ch {
            '\'' | '"' => quote = Some(ch),
            '{' => brace_depth += 1,
            '}' => brace_depth = brace_depth.saturating_sub(1),
            '[' => bracket_depth += 1,
            ']' => bracket_depth = bracket_depth.saturating_sub(1),
            ':' | '=' if brace_depth == 0 && bracket_depth == 0 => {
                let name = entry[..index].trim();
                let value = entry[index + 1..].trim();
                if name.is_empty() || value.is_empty() {
                    break;
                }
                return Ok((name, value));
            }
            _ => {}
        }
    }
    Err(Error::new(format!(
        "{}:{} nested default entry must look like field: value, got {entry:?}",
        owner.display(),
        line_no
    )))
}

fn validate_literal_constraint(
    literal: &Literal,
    constraint: &Constraint,
    owner: &Path,
    line_no: usize,
) -> Result<(), Error> {
    match (literal, constraint) {
        (Literal::String(text), Constraint::StringMaxLen(max)) if text.len() > *max => {
            Err(Error::new(format!(
                "{}:{} string default exceeds bound {}",
                owner.display(),
                line_no,
                max
            )))
        }
        (Literal::Array(items), Constraint::ArrayMaxLen(max)) if items.len() > *max => {
            Err(Error::new(format!(
                "{}:{} array default exceeds bound {}",
                owner.display(),
                line_no,
                max
            )))
        }
        _ => Ok(()),
    }
}
