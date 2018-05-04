/*
 * Summary: error handling
 * Description: the API used to report errors
 *
 * Copy: See Copyright for the status of this software.
 *
 * Author: Daniel Veillard
 */

#include <libxml/parser.h>

#ifndef __XML_ERROR_H__
#define __XML_ERROR_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * xmlErrorLevel:
 *
 * Indicates the level of an error
 */
typedef enum {
    XML_ERR_NONE = 0,
    XML_ERR_WARNING = 1,	/* A simple warning */
    XML_ERR_ERROR = 2,		/* A recoverable error */
    XML_ERR_FATAL = 3		/* A fatal error */
} xmlErrorLevel;

/**
 * xmlErrorDomain:
 *
 * Indicates where an error may have come from
 */
typedef enum {
    XML_FROM_NONE = 0,
    XML_FROM_PARSER,	/* The XML parser */
    XML_FROM_TREE,	/* The tree module */
    XML_FROM_NAMESPACE,	/* The XML Namespace module */
    XML_FROM_DTD,	/* The XML DTD validation with parser context*/
    XML_FROM_HTML,	/* The HTML parser */
    XML_FROM_MEMORY,	/* The memory allocator */
    XML_FROM_OUTPUT,	/* The serialization code */
    XML_FROM_IO,	/* The Input/Output stack */
    XML_FROM_FTP,	/* The FTP module */
    XML_FROM_HTTP,	/* The HTTP module */
    XML_FROM_XINCLUDE,	/* The XInclude processing */
    XML_FROM_XPATH,	/* The XPath module */
    XML_FROM_XPOINTER,	/* The XPointer module */
    XML_FROM_REGEXP,	/* The regular expressions module */
    XML_FROM_DATATYPE,	/* The W3C XML Schemas Datatype module */
    XML_FROM_SCHEMASP,	/* The W3C XML Schemas parser module */
    XML_FROM_SCHEMASV,	/* The W3C XML Schemas validation module */
    XML_FROM_RELAXNGP,	/* The Relax-NG parser module */
    XML_FROM_RELAXNGV,	/* The Relax-NG validator module */
    XML_FROM_CATALOG,	/* The Catalog module */
    XML_FROM_C14N,	/* The Canonicalization module */
    XML_FROM_XSLT,	/* The XSLT engine from libxslt */
    XML_FROM_VALID,	/* The XML DTD validation with valid context */
    XML_FROM_CHECK,	/* The error checking module */
    XML_FROM_WRITER,	/* The xmlwriter module */
    XML_FROM_MODULE,	/* The dynamically loaded module module*/
    XML_FROM_I18N,	/* The module handling character conversion */
    XML_FROM_SCHEMATRONV,/* The Schematron validator module */
    XML_FROM_BUFFER,    /* The buffers module */
    XML_FROM_URI        /* The URI module */
} xmlErrorDomain;

/**
 * xmlError:
 *
 * An XML Error instance.
 */

typedef struct _xmlError xmlError;
typedef xmlError *xmlErrorPtr;
struct _xmlError {
    int		domain;	/* What part of the library raised this error */
    int		code;	/* The error code, e.g. an xmlParserError */
    char       *message;/* human-readable informative error message */
    xmlErrorLevel level;/* how consequent is the error */
    char       *file;	/* the filename */
    int		line;	/* the line number if available */
    char       *str1;	/* extra string information */
    char       *str2;	/* extra string information */
    char       *str3;	/* extra string information */
    int		int1;	/* extra number information */
    int		int2;	/* error column # or 0 if N/A (todo: rename field when we would brk ABI) */
    void       *ctxt;   /* the parser context if available */
    void       *node;   /* the node in the tree */
};

/**
 * xmlParserError:
 *
 * This is an error that the XML (or HTML) parser can generate
 */
typedef enum {
    XML_ERR_OK = 0,
    XML_ERR_INTERNAL_ERROR, /* 1 */
    XML_ERR_NO_MEMORY, /* 2 */
    XML_ERR_DOCUMENT_START, /* 3 */
    XML_ERR_DOCUMENT_EMPTY, /* 4 */
    XML_ERR_DOCUMENT_END, /* 5 */
    XML_ERR_INVALID_HEX_CHARREF, /* 6 */
    XML_ERR_INVALID_DEC_CHARREF, /* 7 */
    XML_ERR_INVALID_CHARREF, /* 8 */
    XML_ERR_INVALID_CHAR, /* 9 */
    XML_ERR_CHARREF_AT_EOF, /* 10 */
    XML_ERR_CHARREF_IN_PROLOG, /* 11 */
    XML_ERR_CHARREF_IN_EPILOG, /* 12 */
    XML_ERR_CHARREF_IN_DTD, /* 13 */
    XML_ERR_ENTITYREF_AT_EOF, /* 14 */
    XML_ERR_ENTITYREF_IN_PROLOG, /* 15 */
    XML_ERR_ENTITYREF_IN_EPILOG, /* 16 */
    XML_ERR_ENTITYREF_IN_DTD, /* 17 */
    XML_ERR_PEREF_AT_EOF, /* 18 */
    XML_ERR_PEREF_IN_PROLOG, /* 19 */
    XML_ERR_PEREF_IN_EPILOG, /* 20 */
    XML_ERR_PEREF_IN_INT_SUBSET, /* 21 */
    XML_ERR_ENTITYREF_NO_NAME, /* 22 */
    XML_ERR_ENTITYREF_SEMICOL_MISSING, /* 23 */
    XML_ERR_PEREF_NO_NAME, /* 24 */
    XML_ERR_PEREF_SEMICOL_MISSING, /* 25 */
    XML_ERR_UNDECLARED_ENTITY, /* 26 */
    XML_WAR_UNDECLARED_ENTITY, /* 27 */
    XML_ERR_UNPARSED_ENTITY, /* 28 */
    XML_ERR_ENTITY_IS_EXTERNAL, /* 29 */
    XML_ERR_ENTITY_IS_PARAMETER, /* 30 */
    XML_ERR_UNKNOWN_ENCODING, /* 31 */
    XML_ERR_UNSUPPORTED_ENCODING, /* 32 */
    XML_ERR_STRING_NOT_STARTED, /* 33 */
    XML_ERR_STRING_NOT_CLOSED, /* 34 */
    XML_ERR_NS_DECL_ERROR, /* 35 */
    XML_ERR_ENTITY_NOT_STARTED, /* 36 */
    XML_ERR_ENTITY_NOT_FINISHED, /* 37 */
    XML_ERR_LT_IN_ATTRIBUTE, /* 38 */
    XML_ERR_ATTRIBUTE_NOT_STARTED, /* 39 */
    XML_ERR_ATTRIBUTE_NOT_FINISHED, /* 40 */
    XML_ERR_ATTRIBUTE_WITHOUT_VALUE, /* 41 */
    XML_ERR_ATTRIBUTE_REDEFINED, /* 42 */
    XML_ERR_LITERAL_NOT_STARTED, /* 43 */
    XML_ERR_LITERAL_NOT_FINISHED, /* 44 */
    XML_ERR_COMMENT_NOT_FINISHED, /* 45 */
    XML_ERR_PI_NOT_STARTED, /* 46 */
    XML_ERR_PI_NOT_FINISHED, /* 47 */
    XML_ERR_NOTATION_NOT_STARTED, /* 48 */
    XML_ERR_NOTATION_NOT_FINISHED, /* 49 */
    XML_ERR_ATTLIST_NOT_STARTED, /* 50 */
    XML_ERR_ATTLIST_NOT_FINISHED, /* 51 */
    XML_ERR_MIXED_NOT_STARTED, /* 52 */
    XML_ERR_MIXED_NOT_FINISHED, /* 53 */
    XML_ERR_ELEMCONTENT_NOT_STARTED, /* 54 */
    XML_ERR_ELEMCONTENT_NOT_FINISHED, /* 55 */
    XML_ERR_XMLDECL_NOT_STARTED, /* 56 */
    XML_ERR_XMLDECL_NOT_FINISHED, /* 57 */
    XML_ERR_CONDSEC_NOT_STARTED, /* 58 */
    XML_ERR_CONDSEC_NOT_FINISHED, /* 59 */
    XML_ERR_EXT_SUBSET_NOT_FINISHED, /* 60 */
    XML_ERR_DOCTYPE_NOT_FINISHED, /* 61 */
    XML_ERR_MISPLACED_CDATA_END, /* 62 */
    XML_ERR_CDATA_NOT_FINISHED, /* 63 */
    XML_ERR_RESERVED_XML_NAME, /* 64 */
    XML_ERR_SPACE_REQUIRED, /* 65 */
    XML_ERR_SEPARATOR_REQUIRED, /* 66 */
    XML_ERR_NMTOKEN_REQUIRED, /* 67 */
    XML_ERR_NAME_REQUIRED, /* 68 */
    XML_ERR_PCDATA_REQUIRED, /* 69 */
    XML_ERR_URI_REQUIRED, /* 70 */
    XML_ERR_PUBID_REQUIRED, /* 71 */
    XML_ERR_LT_REQUIRED, /* 72 */
    XML_ERR_GT_REQUIRED, /* 73 */
    XML_ERR_LTSLASH_REQUIRED, /* 74 */
    XML_ERR_EQUAL_REQUIRED, /* 75 */
    XML_ERR_TAG_NAME_MISMATCH, /* 76 */
    XML_ERR_TAG_NOT_FINISHED, /* 77 */
    XML_ERR_STANDALONE_VALUE, /* 78 */
    XML_ERR_ENCODING_NAME, /* 79 */
    XML_ERR_HYPHEN_IN_COMMENT, /* 80 */
    XML_ERR_INVALID_ENCODING, /* 81 */
    XML_ERR_EXT_ENTITY_STANDALONE, /* 82 */
    XML_ERR_CONDSEC_INVALID, /* 83 */
    XML_ERR_VALUE_REQUIRED, /* 84 */
    XML_ERR_NOT_WELL_BALANCED, /* 85 */
    XML_ERR_EXTRA_CONTENT, /* 86 */
    XML_ERR_ENTITY_CHAR_ERROR, /* 87 */
    XML_ERR_ENTITY_PE_INTERNAL, /* 88 */
    XML_ERR_ENTITY_LOOP, /* 89 */
    XML_ERR_ENTITY_BOUNDARY, /* 90 */
    XML_ERR_INVALID_URI, /* 91 */
    XML_ERR_URI_FRAGMENT, /* 92 */
    XML_WAR_CATALOG_PI, /* 93 */
    XML_ERR_NO_DTD, /* 94 */
    XML_ERR_CONDSEC_INVALID_KEYWORD, /* 95 */
    XML_ERR_VERSION_MISSING, /* 96 */
    XML_WAR_UNKNOWN_VERSION, /* 97 */
    XML_WAR_LANG_VALUE, /* 98 */
    XML_WAR_NS_URI, /* 99 */
    XML_WAR_NS_URI_RELATIVE, /* 100 */
    XML_ERR_MISSING_ENCODING, /* 101 */
    XML_WAR_SPACE_VALUE, /* 102 */
    XML_ERR_NOT_STANDALONE, /* 103 */
    XML_ERR_ENTITY_PROCESSING, /* 104 */
    XML_ERR_NOTATION_PROCESSING, /* 105 */
    XML_WAR_NS_COLUMN, /* 106 */
    XML_WAR_ENTITY_REDEFINED, /* 107 */
    XML_ERR_UNKNOWN_VERSION, /* 108 */
    XML_ERR_VERSION_MISMATCH, /* 109 */
    XML_ERR_NAME_TOO_LONG, /* 110 */
    XML_ERR_USER_STOP, /* 111 */
    XML_NS_ERR_XML_NAMESPACE = 200,
    XML_NS_ERR_UNDEFINED_NAMESPACE, /* 201 */
    XML_NS_ERR_QNAME, /* 202 */
    XML_NS_ERR_ATTRIBUTE_REDEFINED, /* 203 */
    XML_NS_ERR_EMPTY, /* 204 */
    XML_NS_ERR_COLON, /* 205 */
    XML_DTD_ATTRIBUTE_DEFAULT = 500,
    XML_DTD_ATTRIBUTE_REDEFINED, /* 501 */
    XML_DTD_ATTRIBUTE_VALUE, /* 502 */
    XML_DTD_CONTENT_ERROR, /* 503 */
    XML_DTD_CONTENT_MODEL, /* 504 */
    XML_DTD_CONTENT_NOT_DETERMINIST, /* 505 */
    XML_DTD_DIFFERENT_PREFIX, /* 506 */
    XML_DTD_ELEM_DEFAULT_NAMESPACE, /* 507 */
    XML_DTD_ELEM_NAMESPACE, /* 508 */
    XML_DTD_ELEM_REDEFINED, /* 509 */
    XML_DTD_EMPTY_NOTATION, /* 510 */
    XML_DTD_ENTITY_TYPE, /* 511 */
    XML_DTD_ID_FIXED, /* 512 */
    XML_DTD_ID_REDEFINED, /* 513 */
    XML_DTD_ID_SUBSET, /* 514 */
    XML_DTD_INVALID_CHILD, /* 515 */
    XML_DTD_INVALID_DEFAULT, /* 516 */
    XML_DTD_LOAD_ERROR, /* 517 */
    XML_DTD_MISSING_ATTRIBUTE, /* 518 */
    XML_DTD_MIXED_CORRUPT, /* 519 */
    XML_DTD_MULTIPLE_ID, /* 520 */
    XML_DTD_NO_DOC, /* 521 */
    XML_DTD_NO_DTD, /* 522 */
    XML_DTD_NO_ELEM_NAME, /* 523 */
    XML_DTD_NO_PREFIX, /* 524 */
    XML_DTD_NO_ROOT, /* 525 */
    XML_DTD_NOTATION_REDEFINED, /* 526 */
    XML_DTD_NOTATION_VALUE, /* 527 */
    XML_DTD_NOT_EMPTY, /* 528 */
    XML_DTD_NOT_PCDATA, /* 529 */
    XML_DTD_NOT_STANDALONE, /* 530 */
    XML_DTD_ROOT_NAME, /* 531 */
    XML_DTD_STANDALONE_WHITE_SPACE, /* 532 */
    XML_DTD_UNKNOWN_ATTRIBUTE, /* 533 */
    XML_DTD_UNKNOWN_ELEM, /* 534 */
    XML_DTD_UNKNOWN_ENTITY, /* 535 */
    XML_DTD_UNKNOWN_ID, /* 536 */
    XML_DTD_UNKNOWN_NOTATION, /* 537 */
    XML_DTD_STANDALONE_DEFAULTED, /* 538 */
    XML_DTD_XMLID_VALUE, /* 539 */
    XML_DTD_XMLID_TYPE, /* 540 */
    XML_DTD_DUP_TOKEN, /* 541 */
    XML_HTML_STRUCURE_ERROR = 800,
    XML_HTML_UNKNOWN_TAG, /* 801 */
    XML_RNGP_ANYNAME_ATTR_ANCESTOR = 1000,
    XML_RNGP_ATTR_CONFLICT, /* 1001 */
    XML_RNGP_ATTRIBUTE_CHILDREN, /* 1002 */
    XML_RNGP_ATTRIBUTE_CONTENT, /* 1003 */
    XML_RNGP_ATTRIBUTE_EMPTY, /* 1004 */
    XML_RNGP_ATTRIBUTE_NOOP, /* 1005 */
    XML_RNGP_CHOICE_CONTENT, /* 1006 */
    XML_RNGP_CHOICE_EMPTY, /* 1007 */
    XML_RNGP_CREATE_FAILURE, /* 1008 */
    XML_RNGP_DATA_CONTENT, /* 1009 */
    XML_RNGP_DEF_CHOICE_AND_INTERLEAVE, /* 1010 */
    XML_RNGP_DEFINE_CREATE_FAILED, /* 1011 */
    XML_RNGP_DEFINE_EMPTY, /* 1012 */
    XML_RNGP_DEFINE_MISSING, /* 1013 */
    XML_RNGP_DEFINE_NAME_MISSING, /* 1014 */
    XML_RNGP_ELEM_CONTENT_EMPTY, /* 1015 */
    XML_RNGP_ELEM_CONTENT_ERROR, /* 1016 */
    XML_RNGP_ELEMENT_EMPTY, /* 1017 */
    XML_RNGP_ELEMENT_CONTENT, /* 1018 */
    XML_RNGP_ELEMENT_NAME, /* 1019 */
    XML_RNGP_ELEMENT_NO_CONTENT, /* 1020 */
    XML_RNGP_ELEM_TEXT_CONFLICT, /* 1021 */
    XML_RNGP_EMPTY, /* 1022 */
    XML_RNGP_EMPTY_CONSTRUCT, /* 1023 */
    XML_RNGP_EMPTY_CONTENT, /* 1024 */
    XML_RNGP_EMPTY_NOT_EMPTY, /* 1025 */
    XML_RNGP_ERROR_TYPE_LIB, /* 1026 */
    XML_RNGP_EXCEPT_EMPTY, /* 1027 */
    XML_RNGP_EXCEPT_MISSING, /* 1028 */
    XML_RNGP_EXCEPT_MULTIPLE, /* 1029 */
    XML_RNGP_EXCEPT_NO_CONTENT, /* 1030 */
    XML_RNGP_EXTERNALREF_EMTPY, /* 1031 */
    XML_RNGP_EXTERNAL_REF_FAILURE, /* 1032 */
    XML_RNGP_EXTERNALREF_RECURSE, /* 1033 */
    XML_RNGP_FORBIDDEN_ATTRIBUTE, /* 1034 */
    XML_RNGP_FOREIGN_ELEMENT, /* 1035 */
    XML_RNGP_GRAMMAR_CONTENT, /* 1036 */
    XML_RNGP_GRAMMAR_EMPTY, /* 1037 */
    XML_RNGP_GRAMMAR_MISSING, /* 1038 */
    XML_RNGP_GRAMMAR_NO_START, /* 1039 */
    XML_RNGP_GROUP_ATTR_CONFLICT, /* 1040 */
    XML_RNGP_HREF_ERROR, /* 1041 */
    XML_RNGP_INCLUDE_EMPTY, /* 1042 */
    XML_RNGP_INCLUDE_FAILURE, /* 1043 */
    XML_RNGP_INCLUDE_RECURSE, /* 1044 */
    XML_RNGP_INTERLEAVE_ADD, /* 1045 */
    XML_RNGP_INTERLEAVE_CREATE_FAILED, /* 1046 */
    XML_RNGP_INTERLEAVE_EMPTY, /* 1047 */
    XML_RNGP_INTERLEAVE_NO_CONTENT, /* 1048 */
    XML_RNGP_INVALID_DEFINE_NAME, /* 1049 */
    XML_RNGP_INVALID_URI, /* 1050 */
    XML_RNGP_INVALID_VALUE, /* 1051 */
    XML_RNGP_MISSING_HREF, /* 1052 */
    XML_RNGP_NAME_MISSING, /* 1053 */
    XML_RNGP_NEED_COMBINE, /* 1054 */
    XML_RNGP_NOTALLOWED_NOT_EMPTY, /* 1055 */
    XML_RNGP_NSNAME_ATTR_ANCESTOR, /* 1056 */
    XML_RNGP_NSNAME_NO_NS, /* 1057 */
    XML_RNGP_PARAM_FORBIDDEN, /* 1058 */
    XML_RNGP_PARAM_NAME_MISSING, /* 1059 */
    XML_RNGP_PARENTREF_CREATE_FAILED, /* 1060 */
    XML_RNGP_PARENTREF_NAME_INVALID, /* 1061 */
    XML_RNGP_PARENTREF_NO_NAME, /* 1062 */
    XML_RNGP_PARENTREF_NO_PARENT, /* 1063 */
    XML_RNGP_PARENTREF_NOT_EMPTY, /* 1064 */
    XML_RNGP_PARSE_ERROR, /* 1065 */
    XML_RNGP_PAT_ANYNAME_EXCEPT_ANYNAME, /* 1066 */
    XML_RNGP_PAT_ATTR_ATTR, /* 1067 */
    XML_RNGP_PAT_ATTR_ELEM, /* 1068 */
    XML_RNGP_PAT_DATA_EXCEPT_ATTR, /* 1069 */
    XML_RNGP_PAT_DATA_EXCEPT_ELEM, /* 1070 */
    XML_RNGP_PAT_DATA_EXCEPT_EMPTY, /* 1071 */
    XML_RNGP_PAT_DATA_EXCEPT_GROUP, /* 1072 */
    XML_RNGP_PAT_DATA_EXCEPT_INTERLEAVE, /* 1073 */
    XML_RNGP_PAT_DATA_EXCEPT_LIST, /* 1074 */
    XML_RNGP_PAT_DATA_EXCEPT_ONEMORE, /* 1075 */
    XML_RNGP_PAT_DATA_EXCEPT_REF, /* 1076 */
    XML_RNGP_PAT_DATA_EXCEPT_TEXT, /* 1077 */
    XML_RNGP_PAT_LIST_ATTR, /* 1078 */
    XML_RNGP_PAT_LIST_ELEM, /* 1079 */
    XML_RNGP_PAT_LIST_INTERLEAVE, /* 1080 */
    XML_RNGP_PAT_LIST_LIST, /* 1081 */
    XML_RNGP_PAT_LIST_REF, /* 1082 */
    XML_RNGP_PAT_LIST_TEXT, /* 1083 */
    XML_RNGP_PAT_NSNAME_EXCEPT_ANYNAME, /* 1084 */
    XML_RNGP_PAT_NSNAME_EXCEPT_NSNAME, /* 1085 */
    XML_RNGP_PAT_ONEMORE_GROUP_ATTR, /* 1086 */
    XML_RNGP_PAT_ONEMORE_INTERLEAVE_ATTR, /* 1087 */
    XML_RNGP_PAT_START_ATTR, /* 1088 */
    XML_RNGP_PAT_START_DATA, /* 1089 */
    XML_RNGP_PAT_START_EMPTY, /* 1090 */
    XML_RNGP_PAT_START_GROUP, /* 1091 */
    XML_RNGP_PAT_START_INTERLEAVE, /* 1092 */
    XML_RNGP_PAT_START_LIST, /* 1093 */
    XML_RNGP_PAT_START_ONEMORE, /* 1094 */
    XML_RNGP_PAT_START_TEXT, /* 1095 */
    XML_RNGP_PAT_START_VALUE, /* 1096 */
    XML_RNGP_PREFIX_UNDEFINED, /* 1097 */
    XML_RNGP_REF_CREATE_FAILED, /* 1098 */
    XML_RNGP_REF_CYCLE, /* 1099 */
    XML_RNGP_REF_NAME_INVALID, /* 1100 */
    XML_RNGP_REF_NO_DEF, /* 1101 */
    XML_RNGP_REF_NO_NAME, /* 1102 */
    XML_RNGP_REF_NOT_EMPTY, /* 1103 */
    XML_RNGP_START_CHOICE_AND_INTERLEAVE, /* 1104 */
    XML_RNGP_START_CONTENT, /* 1105 */
    XML_RNGP_START_EMPTY, /* 1106 */
    XML_RNGP_START_MISSING, /* 1107 */
    XML_RNGP_TEXT_EXPECTED, /* 1108 */
    XML_RNGP_TEXT_HAS_CHILD, /* 1109 */
    XML_RNGP_TYPE_MISSING, /* 1110 */
    XML_RNGP_TYPE_NOT_FOUND, /* 1111 */
    XML_RNGP_TYPE_VALUE, /* 1112 */
    XML_RNGP_UNKNOWN_ATTRIBUTE, /* 1113 */
    XML_RNGP_UNKNOWN_COMBINE, /* 1114 */
    XML_RNGP_UNKNOWN_CONSTRUCT, /* 1115 */
    XML_RNGP_UNKNOWN_TYPE_LIB, /* 1116 */
    XML_RNGP_URI_FRAGMENT, /* 1117 */
    XML_RNGP_URI_NOT_ABSOLUTE, /* 1118 */
    XML_RNGP_VALUE_EMPTY, /* 1119 */
    XML_RNGP_VALUE_NO_CONTENT, /* 1120 */
    XML_RNGP_XMLNS_NAME, /* 1121 */
    XML_RNGP_XML_NS, /* 1122 */
    XML_XPATH_EXPRESSION_OK = 1200,
    XML_XPATH_NUMBER_ERROR, /* 1201 */
    XML_XPATH_UNFINISHED_LITERAL_ERROR, /* 1202 */
    XML_XPATH_START_LITERAL_ERROR, /* 1203 */
    XML_XPATH_VARIABLE_REF_ERROR, /* 1204 */
    XML_XPATH_UNDEF_VARIABLE_ERROR, /* 1205 */
    XML_XPATH_INVALID_PREDICATE_ERROR, /* 1206 */
    XML_XPATH_EXPR_ERROR, /* 1207 */
    XML_XPATH_UNCLOSED_ERROR, /* 1208 */
    XML_XPATH_UNKNOWN_FUNC_ERROR, /* 1209 */
    XML_XPATH_INVALID_OPERAND, /* 1210 */
    XML_XPATH_INVALID_TYPE, /* 1211 */
    XML_XPATH_INVALID_ARITY, /* 1212 */
    XML_XPATH_INVALID_CTXT_SIZE, /* 1213 */
    XML_XPATH_INVALID_CTXT_POSITION, /* 1214 */
    XML_XPATH_MEMORY_ERROR, /* 1215 */
    XML_XPTR_SYNTAX_ERROR, /* 1216 */
    XML_XPTR_RESOURCE_ERROR, /* 1217 */
    XML_XPTR_SUB_RESOURCE_ERROR, /* 1218 */
    XML_XPATH_UNDEF_PREFIX_ERROR, /* 1219 */
    XML_XPATH_ENCODING_ERROR, /* 1220 */
    XML_XPATH_INVALID_CHAR_ERROR, /* 1221 */
    XML_TREE_INVALID_HEX = 1300,
    XML_TREE_INVALID_DEC, /* 1301 */
    XML_TREE_UNTERMINATED_ENTITY, /* 1302 */
    XML_TREE_NOT_UTF8, /* 1303 */
    XML_SAVE_NOT_UTF8 = 1400,
    XML_SAVE_CHAR_INVALID, /* 1401 */
    XML_SAVE_NO_DOCTYPE, /* 1402 */
    XML_SAVE_UNKNOWN_ENCODING, /* 1403 */
    XML_REGEXP_COMPILE_ERROR = 1450,
    XML_IO_UNKNOWN = 1500,
    XML_IO_EACCES, /* 1501 */
    XML_IO_EAGAIN, /* 1502 */
    XML_IO_EBADF, /* 1503 */
    XML_IO_EBADMSG, /* 1504 */
    XML_IO_EBUSY, /* 1505 */
    XML_IO_ECANCELED, /* 1506 */
    XML_IO_ECHILD, /* 1507 */
    XML_IO_EDEADLK, /* 1508 */
    XML_IO_EDOM, /* 1509 */
    XML_IO_EEXIST, /* 1510 */
    XML_IO_EFAULT, /* 1511 */
    XML_IO_EFBIG, /* 1512 */
    XML_IO_EINPROGRESS, /* 1513 */
    XML_IO_EINTR, /* 1514 */
    XML_IO_EINVAL, /* 1515 */
    XML_IO_EIO, /* 1516 */
    XML_IO_EISDIR, /* 1517 */
    XML_IO_EMFILE, /* 1518 */
    XML_IO_EMLINK, /* 1519 */
    XML_IO_EMSGSIZE, /* 1520 */
    XML_IO_ENAMETOOLONG, /* 1521 */
    XML_IO_ENFILE, /* 1522 */
    XML_IO_ENODEV, /* 1523 */
    XML_IO_ENOENT, /* 1524 */
    XML_IO_ENOEXEC, /* 1525 */
    XML_IO_ENOLCK, /* 1526 */
    XML_IO_ENOMEM, /* 1527 */
    XML_IO_ENOSPC, /* 1528 */
    XML_IO_ENOSYS, /* 1529 */
    XML_IO_ENOTDIR, /* 1530 */
    XML_IO_ENOTEMPTY, /* 1531 */
    XML_IO_ENOTSUP, /* 1532 */
    XML_IO_ENOTTY, /* 1533 */
    XML_IO_ENXIO, /* 1534 */
    XML_IO_EPERM, /* 1535 */
    XML_IO_EPIPE, /* 1536 */
    XML_IO_ERANGE, /* 1537 */
    XML_IO_EROFS, /* 1538 */
    XML_IO_ESPIPE, /* 1539 */
    XML_IO_ESRCH, /* 1540 */
    XML_IO_ETIMEDOUT, /* 1541 */
    XML_IO_EXDEV, /* 1542 */
    XML_IO_NETWORK_ATTEMPT, /* 1543 */
    XML_IO_ENCODER, /* 1544 */
    XML_IO_FLUSH, /* 1545 */
    XML_IO_WRITE, /* 1546 */
    XML_IO_NO_INPUT, /* 1547 */
    XML_IO_BUFFER_FULL, /* 1548 */
    XML_IO_LOAD_ERROR, /* 1549 */
    XML_IO_ENOTSOCK, /* 1550 */
    XML_IO_EISCONN, /* 1551 */
    XML_IO_ECONNREFUSED, /* 1552 */
    XML_IO_ENETUNREACH, /* 1553 */
    XML_IO_EADDRINUSE, /* 1554 */
    XML_IO_EALREADY, /* 1555 */
    XML_IO_EAFNOSUPPORT, /* 1556 */
    XML_XINCLUDE_RECURSION=1600,
    XML_XINCLUDE_PARSE_VALUE, /* 1601 */
    XML_XINCLUDE_ENTITY_DEF_MISMATCH, /* 1602 */
    XML_XINCLUDE_NO_HREF, /* 1603 */
    XML_XINCLUDE_NO_FALLBACK, /* 1604 */
    XML_XINCLUDE_HREF_URI, /* 1605 */
    XML_XINCLUDE_TEXT_FRAGMENT, /* 1606 */
    XML_XINCLUDE_TEXT_DOCUMENT, /* 1607 */
    XML_XINCLUDE_INVALID_CHAR, /* 1608 */
    XML_XINCLUDE_BUILD_FAILED, /* 1609 */
    XML_XINCLUDE_UNKNOWN_ENCODING, /* 1610 */
    XML_XINCLUDE_MULTIPLE_ROOT, /* 1611 */
    XML_XINCLUDE_XPTR_FAILED, /* 1612 */
    XML_XINCLUDE_XPTR_RESULT, /* 1613 */
    XML_XINCLUDE_INCLUDE_IN_INCLUDE, /* 1614 */
    XML_XINCLUDE_FALLBACKS_IN_INCLUDE, /* 1615 */
    XML_XINCLUDE_FALLBACK_NOT_IN_INCLUDE, /* 1616 */
    XML_XINCLUDE_DEPRECATED_NS, /* 1617 */
    XML_XINCLUDE_FRAGMENT_ID, /* 1618 */
    XML_CATALOG_MISSING_ATTR = 1650,
    XML_CATALOG_ENTRY_BROKEN, /* 1651 */
    XML_CATALOG_PREFER_VALUE, /* 1652 */
    XML_CATALOG_NOT_CATALOG, /* 1653 */
    XML_CATALOG_RECURSION, /* 1654 */
    XML_SCHEMAP_PREFIX_UNDEFINED = 1700,
    XML_SCHEMAP_ATTRFORMDEFAULT_VALUE, /* 1701 */
    XML_SCHEMAP_ATTRGRP_NONAME_NOREF, /* 1702 */
    XML_SCHEMAP_ATTR_NONAME_NOREF, /* 1703 */
    XML_SCHEMAP_COMPLEXTYPE_NONAME_NOREF, /* 1704 */
    XML_SCHEMAP_ELEMFORMDEFAULT_VALUE, /* 1705 */
    XML_SCHEMAP_ELEM_NONAME_NOREF, /* 1706 */
    XML_SCHEMAP_EXTENSION_NO_BASE, /* 1707 */
    XML_SCHEMAP_FACET_NO_VALUE, /* 1708 */
    XML_SCHEMAP_FAILED_BUILD_IMPORT, /* 1709 */
    XML_SCHEMAP_GROUP_NONAME_NOREF, /* 1710 */
    XML_SCHEMAP_IMPORT_NAMESPACE_NOT_URI, /* 1711 */
    XML_SCHEMAP_IMPORT_REDEFINE_NSNAME, /* 1712 */
    XML_SCHEMAP_IMPORT_SCHEMA_NOT_URI, /* 1713 */
    XML_SCHEMAP_INVALID_BOOLEAN, /* 1714 */
    XML_SCHEMAP_INVALID_ENUM, /* 1715 */
    XML_SCHEMAP_INVALID_FACET, /* 1716 */
    XML_SCHEMAP_INVALID_FACET_VALUE, /* 1717 */
    XML_SCHEMAP_INVALID_MAXOCCURS, /* 1718 */
    XML_SCHEMAP_INVALID_MINOCCURS, /* 1719 */
    XML_SCHEMAP_INVALID_REF_AND_SUBTYPE, /* 1720 */
    XML_SCHEMAP_INVALID_WHITE_SPACE, /* 1721 */
    XML_SCHEMAP_NOATTR_NOREF, /* 1722 */
    XML_SCHEMAP_NOTATION_NO_NAME, /* 1723 */
    XML_SCHEMAP_NOTYPE_NOREF, /* 1724 */
    XML_SCHEMAP_REF_AND_SUBTYPE, /* 1725 */
    XML_SCHEMAP_RESTRICTION_NONAME_NOREF, /* 1726 */
    XML_SCHEMAP_SIMPLETYPE_NONAME, /* 1727 */
    XML_SCHEMAP_TYPE_AND_SUBTYPE, /* 1728 */
    XML_SCHEMAP_UNKNOWN_ALL_CHILD, /* 1729 */
    XML_SCHEMAP_UNKNOWN_ANYATTRIBUTE_CHILD, /* 1730 */
    XML_SCHEMAP_UNKNOWN_ATTR_CHILD, /* 1731 */
    XML_SCHEMAP_UNKNOWN_ATTRGRP_CHILD, /* 1732 */
    XML_SCHEMAP_UNKNOWN_ATTRIBUTE_GROUP, /* 1733 */
    XML_SCHEMAP_UNKNOWN_BASE_TYPE, /* 1734 */
    XML_SCHEMAP_UNKNOWN_CHOICE_CHILD, /* 1735 */
    XML_SCHEMAP_UNKNOWN_COMPLEXCONTENT_CHILD, /* 1736 */
    XML_SCHEMAP_UNKNOWN_COMPLEXTYPE_CHILD, /* 1737 */
    XML_SCHEMAP_UNKNOWN_ELEM_CHILD, /* 1738 */
    XML_SCHEMAP_UNKNOWN_EXTENSION_CHILD, /* 1739 */
    XML_SCHEMAP_UNKNOWN_FACET_CHILD, /* 1740 */
    XML_SCHEMAP_UNKNOWN_FACET_TYPE, /* 1741 */
    XML_SCHEMAP_UNKNOWN_GROUP_CHILD, /* 1742 */
    XML_SCHEMAP_UNKNOWN_IMPORT_CHILD, /* 1743 */
    XML_SCHEMAP_UNKNOWN_LIST_CHILD, /* 1744 */
    XML_SCHEMAP_UNKNOWN_NOTATION_CHILD, /* 1745 */
    XML_SCHEMAP_UNKNOWN_PROCESSCONTENT_CHILD, /* 1746 */
    XML_SCHEMAP_UNKNOWN_REF, /* 1747 */
    XML_SCHEMAP_UNKNOWN_RESTRICTION_CHILD, /* 1748 */
    XML_SCHEMAP_UNKNOWN_SCHEMAS_CHILD, /* 1749 */
    XML_SCHEMAP_UNKNOWN_SEQUENCE_CHILD, /* 1750 */
    XML_SCHEMAP_UNKNOWN_SIMPLECONTENT_CHILD, /* 1751 */
    XML_SCHEMAP_UNKNOWN_SIMPLETYPE_CHILD, /* 1752 */
    XML_SCHEMAP_UNKNOWN_TYPE, /* 1753 */
    XML_SCHEMAP_UNKNOWN_UNION_CHILD, /* 1754 */
    XML_SCHEMAP_ELEM_DEFAULT_FIXED, /* 1755 */
    XML_SCHEMAP_REGEXP_INVALID, /* 1756 */
    XML_SCHEMAP_FAILED_LOAD, /* 1757 */
    XML_SCHEMAP_NOTHING_TO_PARSE, /* 1758 */
    XML_SCHEMAP_NOROOT, /* 1759 */
    XML_SCHEMAP_REDEFINED_GROUP, /* 1760 */
    XML_SCHEMAP_REDEFINED_TYPE, /* 1761 */
    XML_SCHEMAP_REDEFINED_ELEMENT, /* 1762 */
    XML_SCHEMAP_REDEFINED_ATTRGROUP, /* 1763 */
    XML_SCHEMAP_REDEFINED_ATTR, /* 1764 */
    XML_SCHEMAP_REDEFINED_NOTATION, /* 1765 */
    XML_SCHEMAP_FAILED_PARSE, /* 1766 */
    XML_SCHEMAP_UNKNOWN_PREFIX, /* 1767 */
    XML_SCHEMAP_DEF_AND_PREFIX, /* 1768 */
    XML_SCHEMAP_UNKNOWN_INCLUDE_CHILD, /* 1769 */
    XML_SCHEMAP_INCLUDE_SCHEMA_NOT_URI, /* 1770 */
    XML_SCHEMAP_INCLUDE_SCHEMA_NO_URI, /* 1771 */
    XML_SCHEMAP_NOT_SCHEMA, /* 1772 */
    XML_SCHEMAP_UNKNOWN_MEMBER_TYPE, /* 1773 */
    XML_SCHEMAP_INVALID_ATTR_USE, /* 1774 */
    XML_SCHEMAP_RECURSIVE, /* 1775 */
    XML_SCHEMAP_SUPERNUMEROUS_LIST_ITEM_TYPE, /* 1776 */
    XML_SCHEMAP_INVALID_ATTR_COMBINATION, /* 1777 */
    XML_SCHEMAP_INVALID_ATTR_INLINE_COMBINATION, /* 1778 */
    XML_SCHEMAP_MISSING_SIMPLETYPE_CHILD, /* 1779 */
    XML_SCHEMAP_INVALID_ATTR_NAME, /* 1780 */
    XML_SCHEMAP_REF_AND_CONTENT, /* 1781 */
    XML_SCHEMAP_CT_PROPS_CORRECT_1, /* 1782 */
    XML_SCHEMAP_CT_PROPS_CORRECT_2, /* 1783 */
    XML_SCHEMAP_CT_PROPS_CORRECT_3, /* 1784 */
    XML_SCHEMAP_CT_PROPS_CORRECT_4, /* 1785 */
    XML_SCHEMAP_CT_PROPS_CORRECT_5, /* 1786 */
    XML_SCHEMAP_DERIVATION_OK_RESTRICTION_1, /* 1787 */
    XML_SCHEMAP_DERIVATION_OK_RESTRICTION_2_1_1, /* 1788 */
    XML_SCHEMAP_DERIVATION_OK_RESTRICTION_2_1_2, /* 1789 */
    XML_SCHEMAP_DERIVATION_OK_RESTRICTION_2_2, /* 1790 */
    XML_SCHEMAP_DERIVATION_OK_RESTRICTION_3, /* 1791 */
    XML_SCHEMAP_WILDCARD_INVALID_NS_MEMBER, /* 1792 */
    XML_SCHEMAP_INTERSECTION_NOT_EXPRESSIBLE, /* 1793 */
    XML_SCHEMAP_UNION_NOT_EXPRESSIBLE, /* 1794 */
    XML_SCHEMAP_SRC_IMPORT_3_1, /* 1795 */
    XML_SCHEMAP_SRC_IMPORT_3_2, /* 1796 */
    XML_SCHEMAP_DERIVATION_OK_RESTRICTION_4_1, /* 1797 */
    XML_SCHEMAP_DERIVATION_OK_RESTRICTION_4_2, /* 1798 */
    XML_SCHEMAP_DERIVATION_OK_RESTRICTION_4_3, /* 1799 */
    XML_SCHEMAP_COS_CT_EXTENDS_1_3, /* 1800 */
    XML_SCHEMAV_NOROOT = 1801,
    XML_SCHEMAV_UNDECLAREDELEM, /* 1802 */
    XML_SCHEMAV_NOTTOPLEVEL, /* 1803 */
    XML_SCHEMAV_MISSING, /* 1804 */
    XML_SCHEMAV_WRONGELEM, /* 1805 */
    XML_SCHEMAV_NOTYPE, /* 1806 */
    XML_SCHEMAV_NOROLLBACK, /* 1807 */
    XML_SCHEMAV_ISABSTRACT, /* 1808 */
    XML_SCHEMAV_NOTEMPTY, /* 1809 */
    XML_SCHEMAV_ELEMCONT, /* 1810 */
    XML_SCHEMAV_HAVEDEFAULT, /* 1811 */
    XML_SCHEMAV_NOTNILLABLE, /* 1812 */
    XML_SCHEMAV_EXTRACONTENT, /* 1813 */
    XML_SCHEMAV_INVALIDATTR, /* 1814 */
    XML_SCHEMAV_INVALIDELEM, /* 1815 */
    XML_SCHEMAV_NOTDETERMINIST, /* 1816 */
    XML_SCHEMAV_CONSTRUCT, /* 1817 */
    XML_SCHEMAV_INTERNAL, /* 1818 */
    XML_SCHEMAV_NOTSIMPLE, /* 1819 */
    XML_SCHEMAV_ATTRUNKNOWN, /* 1820 */
    XML_SCHEMAV_ATTRINVALID, /* 1821 */
    XML_SCHEMAV_VALUE, /* 1822 */
    XML_SCHEMAV_FACET, /* 1823 */
    XML_SCHEMAV_CVC_DATATYPE_VALID_1_2_1, /* 1824 */
    XML_SCHEMAV_CVC_DATATYPE_VALID_1_2_2, /* 1825 */
    XML_SCHEMAV_CVC_DATATYPE_VALID_1_2_3, /* 1826 */
    XML_SCHEMAV_CVC_TYPE_3_1_1, /* 1827 */
    XML_SCHEMAV_CVC_TYPE_3_1_2, /* 1828 */
    XML_SCHEMAV_CVC_FACET_VALID, /* 1829 */
    XML_SCHEMAV_CVC_LENGTH_VALID, /* 1830 */
    XML_SCHEMAV_CVC_MINLENGTH_VALID, /* 1831 */
    XML_SCHEMAV_CVC_MAXLENGTH_VALID, /* 1832 */
    XML_SCHEMAV_CVC_MININCLUSIVE_VALID, /* 1833 */
    XML_SCHEMAV_CVC_MAXINCLUSIVE_VALID, /* 1834 */
    XML_SCHEMAV_CVC_MINEXCLUSIVE_VALID, /* 1835 */
    XML_SCHEMAV_CVC_MAXEXCLUSIVE_VALID, /* 1836 */
    XML_SCHEMAV_CVC_TOTALDIGITS_VALID, /* 1837 */
    XML_SCHEMAV_CVC_FRACTIONDIGITS_VALID, /* 1838 */
    XML_SCHEMAV_CVC_PATTERN_VALID, /* 1839 */
    XML_SCHEMAV_CVC_ENUMERATION_VALID, /* 1840 */
    XML_SCHEMAV_CVC_COMPLEX_TYPE_2_1, /* 1841 */
    XML_SCHEMAV_CVC_COMPLEX_TYPE_2_2, /* 1842 */
    XML_SCHEMAV_CVC_COMPLEX_TYPE_2_3, /* 1843 */
    XML_SCHEMAV_CVC_COMPLEX_TYPE_2_4, /* 1844 */
    XML_SCHEMAV_CVC_ELT_1, /* 1845 */
    XML_SCHEMAV_CVC_ELT_2, /* 1846 */
    XML_SCHEMAV_CVC_ELT_3_1, /* 1847 */
    XML_SCHEMAV_CVC_ELT_3_2_1, /* 1848 */
    XML_SCHEMAV_CVC_ELT_3_2_2, /* 1849 */
    XML_SCHEMAV_CVC_ELT_4_1, /* 1850 */
    XML_SCHEMAV_CVC_ELT_4_2, /* 1851 */
    XML_SCHEMAV_CVC_ELT_4_3, /* 1852 */
    XML_SCHEMAV_CVC_ELT_5_1_1, /* 1853 */
    XML_SCHEMAV_CVC_ELT_5_1_2, /* 1854 */
    XML_SCHEMAV_CVC_ELT_5_2_1, /* 1855 */
    XML_SCHEMAV_CVC_ELT_5_2_2_1, /* 1856 */
    XML_SCHEMAV_CVC_ELT_5_2_2_2_1, /* 1857 */
    XML_SCHEMAV_CVC_ELT_5_2_2_2_2, /* 1858 */
    XML_SCHEMAV_CVC_ELT_6, /* 1859 */
    XML_SCHEMAV_CVC_ELT_7, /* 1860 */
    XML_SCHEMAV_CVC_ATTRIBUTE_1, /* 1861 */
    XML_SCHEMAV_CVC_ATTRIBUTE_2, /* 1862 */
    XML_SCHEMAV_CVC_ATTRIBUTE_3, /* 1863 */
    XML_SCHEMAV_CVC_ATTRIBUTE_4, /* 1864 */
    XML_SCHEMAV_CVC_COMPLEX_TYPE_3_1, /* 1865 */
    XML_SCHEMAV_CVC_COMPLEX_TYPE_3_2_1, /* 1866 */
    XML_SCHEMAV_CVC_COMPLEX_TYPE_3_2_2, /* 1867 */
    XML_SCHEMAV_CVC_COMPLEX_TYPE_4, /* 1868 */
    XML_SCHEMAV_CVC_COMPLEX_TYPE_5_1, /* 1869 */
    XML_SCHEMAV_CVC_COMPLEX_TYPE_5_2, /* 1870 */
    XML_SCHEMAV_ELEMENT_CONTENT, /* 1871 */
    XML_SCHEMAV_DOCUMENT_ELEMENT_MISSING, /* 1872 */
    XML_SCHEMAV_CVC_COMPLEX_TYPE_1, /* 1873 */
    XML_SCHEMAV_CVC_AU, /* 1874 */
    XML_SCHEMAV_CVC_TYPE_1, /* 1875 */
    XML_SCHEMAV_CVC_TYPE_2, /* 1876 */
    XML_SCHEMAV_CVC_IDC, /* 1877 */
    XML_SCHEMAV_CVC_WILDCARD, /* 1878 */
    XML_SCHEMAV_MISC, /* 1879 */
    XML_XPTR_UNKNOWN_SCHEME = 1900,
    XML_XPTR_CHILDSEQ_START, /* 1901 */
    XML_XPTR_EVAL_FAILED, /* 1902 */
    XML_XPTR_EXTRA_OBJECTS, /* 1903 */
    XML_C14N_CREATE_CTXT = 1950,
    XML_C14N_REQUIRES_UTF8, /* 1951 */
    XML_C14N_CREATE_STACK, /* 1952 */
    XML_C14N_INVALID_NODE, /* 1953 */
    XML_C14N_UNKNOW_NODE, /* 1954 */
    XML_C14N_RELATIVE_NAMESPACE, /* 1955 */
    XML_FTP_PASV_ANSWER = 2000,
    XML_FTP_EPSV_ANSWER, /* 2001 */
    XML_FTP_ACCNT, /* 2002 */
    XML_FTP_URL_SYNTAX, /* 2003 */
    XML_HTTP_URL_SYNTAX = 2020,
    XML_HTTP_USE_IP, /* 2021 */
    XML_HTTP_UNKNOWN_HOST, /* 2022 */
    XML_SCHEMAP_SRC_SIMPLE_TYPE_1 = 3000,
    XML_SCHEMAP_SRC_SIMPLE_TYPE_2, /* 3001 */
    XML_SCHEMAP_SRC_SIMPLE_TYPE_3, /* 3002 */
    XML_SCHEMAP_SRC_SIMPLE_TYPE_4, /* 3003 */
    XML_SCHEMAP_SRC_RESOLVE, /* 3004 */
    XML_SCHEMAP_SRC_RESTRICTION_BASE_OR_SIMPLETYPE, /* 3005 */
    XML_SCHEMAP_SRC_LIST_ITEMTYPE_OR_SIMPLETYPE, /* 3006 */
    XML_SCHEMAP_SRC_UNION_MEMBERTYPES_OR_SIMPLETYPES, /* 3007 */
    XML_SCHEMAP_ST_PROPS_CORRECT_1, /* 3008 */
    XML_SCHEMAP_ST_PROPS_CORRECT_2, /* 3009 */
    XML_SCHEMAP_ST_PROPS_CORRECT_3, /* 3010 */
    XML_SCHEMAP_COS_ST_RESTRICTS_1_1, /* 3011 */
    XML_SCHEMAP_COS_ST_RESTRICTS_1_2, /* 3012 */
    XML_SCHEMAP_COS_ST_RESTRICTS_1_3_1, /* 3013 */
    XML_SCHEMAP_COS_ST_RESTRICTS_1_3_2, /* 3014 */
    XML_SCHEMAP_COS_ST_RESTRICTS_2_1, /* 3015 */
    XML_SCHEMAP_COS_ST_RESTRICTS_2_3_1_1, /* 3016 */
    XML_SCHEMAP_COS_ST_RESTRICTS_2_3_1_2, /* 3017 */
    XML_SCHEMAP_COS_ST_RESTRICTS_2_3_2_1, /* 3018 */
    XML_SCHEMAP_COS_ST_RESTRICTS_2_3_2_2, /* 3019 */
    XML_SCHEMAP_COS_ST_RESTRICTS_2_3_2_3, /* 3020 */
    XML_SCHEMAP_COS_ST_RESTRICTS_2_3_2_4, /* 3021 */
    XML_SCHEMAP_COS_ST_RESTRICTS_2_3_2_5, /* 3022 */
    XML_SCHEMAP_COS_ST_RESTRICTS_3_1, /* 3023 */
    XML_SCHEMAP_COS_ST_RESTRICTS_3_3_1, /* 3024 */
    XML_SCHEMAP_COS_ST_RESTRICTS_3_3_1_2, /* 3025 */
    XML_SCHEMAP_COS_ST_RESTRICTS_3_3_2_2, /* 3026 */
    XML_SCHEMAP_COS_ST_RESTRICTS_3_3_2_1, /* 3027 */
    XML_SCHEMAP_COS_ST_RESTRICTS_3_3_2_3, /* 3028 */
    XML_SCHEMAP_COS_ST_RESTRICTS_3_3_2_4, /* 3029 */
    XML_SCHEMAP_COS_ST_RESTRICTS_3_3_2_5, /* 3030 */
    XML_SCHEMAP_COS_ST_DERIVED_OK_2_1, /* 3031 */
    XML_SCHEMAP_COS_ST_DERIVED_OK_2_2, /* 3032 */
    XML_SCHEMAP_S4S_ELEM_NOT_ALLOWED, /* 3033 */
    XML_SCHEMAP_S4S_ELEM_MISSING, /* 3034 */
    XML_SCHEMAP_S4S_ATTR_NOT_ALLOWED, /* 3035 */
    XML_SCHEMAP_S4S_ATTR_MISSING, /* 3036 */
    XML_SCHEMAP_S4S_ATTR_INVALID_VALUE, /* 3037 */
    XML_SCHEMAP_SRC_ELEMENT_1, /* 3038 */
    XML_SCHEMAP_SRC_ELEMENT_2_1, /* 3039 */
    XML_SCHEMAP_SRC_ELEMENT_2_2, /* 3040 */
    XML_SCHEMAP_SRC_ELEMENT_3, /* 3041 */
    XML_SCHEMAP_P_PROPS_CORRECT_1, /* 3042 */
    XML_SCHEMAP_P_PROPS_CORRECT_2_1, /* 3043 */
    XML_SCHEMAP_P_PROPS_CORRECT_2_2, /* 3044 */
    XML_SCHEMAP_E_PROPS_CORRECT_2, /* 3045 */
    XML_SCHEMAP_E_PROPS_CORRECT_3, /* 3046 */
    XML_SCHEMAP_E_PROPS_CORRECT_4, /* 3047 */
    XML_SCHEMAP_E_PROPS_CORRECT_5, /* 3048 */
    XML_SCHEMAP_E_PROPS_CORRECT_6, /* 3049 */
    XML_SCHEMAP_SRC_INCLUDE, /* 3050 */
    XML_SCHEMAP_SRC_ATTRIBUTE_1, /* 3051 */
    XML_SCHEMAP_SRC_ATTRIBUTE_2, /* 3052 */
    XML_SCHEMAP_SRC_ATTRIBUTE_3_1, /* 3053 */
    XML_SCHEMAP_SRC_ATTRIBUTE_3_2, /* 3054 */
    XML_SCHEMAP_SRC_ATTRIBUTE_4, /* 3055 */
    XML_SCHEMAP_NO_XMLNS, /* 3056 */
    XML_SCHEMAP_NO_XSI, /* 3057 */
    XML_SCHEMAP_COS_VALID_DEFAULT_1, /* 3058 */
    XML_SCHEMAP_COS_VALID_DEFAULT_2_1, /* 3059 */
    XML_SCHEMAP_COS_VALID_DEFAULT_2_2_1, /* 3060 */
    XML_SCHEMAP_COS_VALID_DEFAULT_2_2_2, /* 3061 */
    XML_SCHEMAP_CVC_SIMPLE_TYPE, /* 3062 */
    XML_SCHEMAP_COS_CT_EXTENDS_1_1, /* 3063 */
    XML_SCHEMAP_SRC_IMPORT_1_1, /* 3064 */
    XML_SCHEMAP_SRC_IMPORT_1_2, /* 3065 */
    XML_SCHEMAP_SRC_IMPORT_2, /* 3066 */
    XML_SCHEMAP_SRC_IMPORT_2_1, /* 3067 */
    XML_SCHEMAP_SRC_IMPORT_2_2, /* 3068 */
    XML_SCHEMAP_INTERNAL, /* 3069 non-W3C */
    XML_SCHEMAP_NOT_DETERMINISTIC, /* 3070 non-W3C */
    XML_SCHEMAP_SRC_ATTRIBUTE_GROUP_1, /* 3071 */
    XML_SCHEMAP_SRC_ATTRIBUTE_GROUP_2, /* 3072 */
    XML_SCHEMAP_SRC_ATTRIBUTE_GROUP_3, /* 3073 */
    XML_SCHEMAP_MG_PROPS_CORRECT_1, /* 3074 */
    XML_SCHEMAP_MG_PROPS_CORRECT_2, /* 3075 */
    XML_SCHEMAP_SRC_CT_1, /* 3076 */
    XML_SCHEMAP_DERIVATION_OK_RESTRICTION_2_1_3, /* 3077 */
    XML_SCHEMAP_AU_PROPS_CORRECT_2, /* 3078 */
    XML_SCHEMAP_A_PROPS_CORRECT_2, /* 3079 */
    XML_SCHEMAP_C_PROPS_CORRECT, /* 3080 */
    XML_SCHEMAP_SRC_REDEFINE, /* 3081 */
    XML_SCHEMAP_SRC_IMPORT, /* 3082 */
    XML_SCHEMAP_WARN_SKIP_SCHEMA, /* 3083 */
    XML_SCHEMAP_WARN_UNLOCATED_SCHEMA, /* 3084 */
    XML_SCHEMAP_WARN_ATTR_REDECL_PROH, /* 3085 */
    XML_SCHEMAP_WARN_ATTR_POINTLESS_PROH, /* 3085 */
    XML_SCHEMAP_AG_PROPS_CORRECT, /* 3086 */
    XML_SCHEMAP_COS_CT_EXTENDS_1_2, /* 3087 */
    XML_SCHEMAP_AU_PROPS_CORRECT, /* 3088 */
    XML_SCHEMAP_A_PROPS_CORRECT_3, /* 3089 */
    XML_SCHEMAP_COS_ALL_LIMITED, /* 3090 */
    XML_SCHEMATRONV_ASSERT = 4000, /* 4000 */
    XML_SCHEMATRONV_REPORT,
    XML_MODULE_OPEN = 4900, /* 4900 */
    XML_MODULE_CLOSE, /* 4901 */
    XML_CHECK_FOUND_ELEMENT = 5000,
    XML_CHECK_FOUND_ATTRIBUTE, /* 5001 */
    XML_CHECK_FOUND_TEXT, /* 5002 */
    XML_CHECK_FOUND_CDATA, /* 5003 */
    XML_CHECK_FOUND_ENTITYREF, /* 5004 */
    XML_CHECK_FOUND_ENTITY, /* 5005 */
    XML_CHECK_FOUND_PI, /* 5006 */
    XML_CHECK_FOUND_COMMENT, /* 5007 */
    XML_CHECK_FOUND_DOCTYPE, /* 5008 */
    XML_CHECK_FOUND_FRAGMENT, /* 5009 */
    XML_CHECK_FOUND_NOTATION, /* 5010 */
    XML_CHECK_UNKNOWN_NODE, /* 5011 */
    XML_CHECK_ENTITY_TYPE, /* 5012 */
    XML_CHECK_NO_PARENT, /* 5013 */
    XML_CHECK_NO_DOC, /* 5014 */
    XML_CHECK_NO_NAME, /* 5015 */
    XML_CHECK_NO_ELEM, /* 5016 */
    XML_CHECK_WRONG_DOC, /* 5017 */
    XML_CHECK_NO_PREV, /* 5018 */
    XML_CHECK_WRONG_PREV, /* 5019 */
    XML_CHECK_NO_NEXT, /* 5020 */
    XML_CHECK_WRONG_NEXT, /* 5021 */
    XML_CHECK_NOT_DTD, /* 5022 */
    XML_CHECK_NOT_ATTR, /* 5023 */
    XML_CHECK_NOT_ATTR_DECL, /* 5024 */
    XML_CHECK_NOT_ELEM_DECL, /* 5025 */
    XML_CHECK_NOT_ENTITY_DECL, /* 5026 */
    XML_CHECK_NOT_NS_DECL, /* 5027 */
    XML_CHECK_NO_HREF, /* 5028 */
    XML_CHECK_WRONG_PARENT,/* 5029 */
    XML_CHECK_NS_SCOPE, /* 5030 */
    XML_CHECK_NS_ANCESTOR, /* 5031 */
    XML_CHECK_NOT_UTF8, /* 5032 */
    XML_CHECK_NO_DICT, /* 5033 */
    XML_CHECK_NOT_NCNAME, /* 5034 */
    XML_CHECK_OUTSIDE_DICT, /* 5035 */
    XML_CHECK_WRONG_NAME, /* 5036 */
    XML_CHECK_NAME_NOT_NULL, /* 5037 */
    XML_I18N_NO_NAME = 6000,
    XML_I18N_NO_HANDLER, /* 6001 */
    XML_I18N_EXCESS_HANDLER, /* 6002 */
    XML_I18N_CONV_FAILED, /* 6003 */
    XML_I18N_NO_OUTPUT, /* 6004 */
    XML_BUF_OVERFLOW = 7000
} xmlParserErrors;

/**
 * xmlGenericErrorFunc:
 * @ctx:  a parsing context
 * @msg:  the message
 * @...:  the extra arguments of the varags to format the message
 *
 * Signature of the function to use when there is an error and
 * no parsing or validity context available .
 */
typedef void (XMLCDECL *xmlGenericErrorFunc) (void *ctx,
				 const char *msg,
				 ...) LIBXML_ATTR_FORMAT(2,3);
/**
 * xmlStructuredErrorFunc:
 * @userData:  user provided data for the error callback
 * @error:  the error being raised.
 *
 * Signature of the function to use when there is an error and
 * the module handles the new error reporting mechanism.
 */
typedef void (XMLCALL *xmlStructuredErrorFunc) (void *userData, xmlErrorPtr error);

/*
 * Use the following function to reset the two global variables
 * xmlGenericError and xmlGenericErrorContext.
 */
XMLPUBFUN void XMLCALL
    xmlSetGenericErrorFunc	(void *ctx,
				 xmlGenericErrorFunc handler);
XMLPUBFUN void XMLCALL
    initGenericErrorDefaultFunc	(xmlGenericErrorFunc *handler);

XMLPUBFUN void XMLCALL
    xmlSetStructuredErrorFunc	(void *ctx,
				 xmlStructuredErrorFunc handler);
/*
 * Default message routines used by SAX and Valid context for error
 * and warning reporting.
 */
XMLPUBFUN void XMLCDECL
    xmlParserError		(void *ctx,
				 const char *msg,
				 ...) LIBXML_ATTR_FORMAT(2,3);
XMLPUBFUN void XMLCDECL
    xmlParserWarning		(void *ctx,
				 const char *msg,
				 ...) LIBXML_ATTR_FORMAT(2,3);
XMLPUBFUN void XMLCDECL
    xmlParserValidityError	(void *ctx,
				 const char *msg,
				 ...) LIBXML_ATTR_FORMAT(2,3);
XMLPUBFUN void XMLCDECL
    xmlParserValidityWarning	(void *ctx,
				 const char *msg,
				 ...) LIBXML_ATTR_FORMAT(2,3);
XMLPUBFUN void XMLCALL
    xmlParserPrintFileInfo	(xmlParserInputPtr input);
XMLPUBFUN void XMLCALL
    xmlParserPrintFileContext	(xmlParserInputPtr input);

/*
 * Extended error information routines
 */
XMLPUBFUN xmlErrorPtr XMLCALL
    xmlGetLastError		(void);
XMLPUBFUN void XMLCALL
    xmlResetLastError		(void);
XMLPUBFUN xmlErrorPtr XMLCALL
    xmlCtxtGetLastError		(void *ctx);
XMLPUBFUN void XMLCALL
    xmlCtxtResetLastError	(void *ctx);
XMLPUBFUN void XMLCALL
    xmlResetError		(xmlErrorPtr err);
XMLPUBFUN int XMLCALL
    xmlCopyError		(xmlErrorPtr from,
				 xmlErrorPtr to);

#ifdef IN_LIBXML
/*
 * Internal callback reporting routine
 */
XMLPUBFUN void XMLCALL
    __xmlRaiseError		(xmlStructuredErrorFunc schannel,
				 xmlGenericErrorFunc channel,
				 void *data,
                                 void *ctx,
				 void *node,
				 int domain,
				 int code,
				 xmlErrorLevel level,
				 const char *file,
				 int line,
				 const char *str1,
				 const char *str2,
				 const char *str3,
				 int int1,
				 int col,
				 const char *msg,
				 ...) LIBXML_ATTR_FORMAT(16,17);
XMLPUBFUN void XMLCALL
    __xmlSimpleError		(int domain,
				 int code,
				 xmlNodePtr node,
				 const char *msg,
				 const char *extra) LIBXML_ATTR_FORMAT(4,0);
#endif
#ifdef __cplusplus
}
#endif
#endif /* __XML_ERROR_H__ */
