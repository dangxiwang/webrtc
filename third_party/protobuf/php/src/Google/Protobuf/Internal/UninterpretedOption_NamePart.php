<?php
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: google/protobuf/descriptor.proto

namespace Google\Protobuf\Internal;

use Google\Protobuf\Internal\GPBType;
use Google\Protobuf\Internal\GPBWire;
use Google\Protobuf\Internal\RepeatedField;
use Google\Protobuf\Internal\InputStream;
use Google\Protobuf\Internal\GPBUtil;

/**
 * The name of the uninterpreted option.  Each string represents a segment in
 * a dot-separated name.  is_extension is true iff a segment represents an
 * extension (denoted with parentheses in options specs in .proto files).
 * E.g.,{ ["foo", false], ["bar.baz", true], ["qux", false] } represents
 * "foo.(bar.baz).qux".
 *
 * Generated from protobuf message <code>google.protobuf.UninterpretedOption.NamePart</code>
 */
class UninterpretedOption_NamePart extends \Google\Protobuf\Internal\Message
{
    /**
     * Generated from protobuf field <code>required string name_part = 1;</code>
     */
    private $name_part = '';
    private $has_name_part = false;
    /**
     * Generated from protobuf field <code>required bool is_extension = 2;</code>
     */
    private $is_extension = false;
    private $has_is_extension = false;

    public function __construct() {
        \GPBMetadata\Google\Protobuf\Internal\Descriptor::initOnce();
        parent::__construct();
    }

    /**
     * Generated from protobuf field <code>required string name_part = 1;</code>
     * @return string
     */
    public function getNamePart()
    {
        return $this->name_part;
    }

    /**
     * Generated from protobuf field <code>required string name_part = 1;</code>
     * @param string $var
     * @return $this
     */
    public function setNamePart($var)
    {
        GPBUtil::checkString($var, True);
        $this->name_part = $var;
        $this->has_name_part = true;

        return $this;
    }

    public function hasNamePart()
    {
        return $this->has_name_part;
    }

    /**
     * Generated from protobuf field <code>required bool is_extension = 2;</code>
     * @return bool
     */
    public function getIsExtension()
    {
        return $this->is_extension;
    }

    /**
     * Generated from protobuf field <code>required bool is_extension = 2;</code>
     * @param bool $var
     * @return $this
     */
    public function setIsExtension($var)
    {
        GPBUtil::checkBool($var);
        $this->is_extension = $var;
        $this->has_is_extension = true;

        return $this;
    }

    public function hasIsExtension()
    {
        return $this->has_is_extension;
    }

}

