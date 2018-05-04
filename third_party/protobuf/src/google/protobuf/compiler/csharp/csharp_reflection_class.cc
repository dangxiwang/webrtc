// Protocol Buffers - Google's data interchange format
// Copyright 2008 Google Inc.  All rights reserved.
// https://developers.google.com/protocol-buffers/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <sstream>

#include <google/protobuf/compiler/code_generator.h>
#include <google/protobuf/compiler/plugin.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/io/printer.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/stubs/strutil.h>


#include <google/protobuf/compiler/csharp/csharp_enum.h>
#include <google/protobuf/compiler/csharp/csharp_helpers.h>
#include <google/protobuf/compiler/csharp/csharp_message.h>
#include <google/protobuf/compiler/csharp/csharp_names.h>
#include <google/protobuf/compiler/csharp/csharp_options.h>
#include <google/protobuf/compiler/csharp/csharp_reflection_class.h>

namespace google {
namespace protobuf {
namespace compiler {
namespace csharp {

ReflectionClassGenerator::ReflectionClassGenerator(const FileDescriptor* file,
                                                   const Options* options)
    : SourceGeneratorBase(file, options),
      file_(file) {
  namespace_ = GetFileNamespace(file);
  reflectionClassname_ = GetReflectionClassUnqualifiedName(file);
}

ReflectionClassGenerator::~ReflectionClassGenerator() {
}

void ReflectionClassGenerator::Generate(io::Printer* printer) {
  WriteIntroduction(printer);

  WriteDescriptor(printer);
  // Close the class declaration.
  printer->Outdent();
  printer->Print("}\n");

  // write children: Enums
  if (file_->enum_type_count() > 0) {
    printer->Print("#region Enums\n");
    for (int i = 0; i < file_->enum_type_count(); i++) {
      EnumGenerator enumGenerator(file_->enum_type(i), this->options());
      enumGenerator.Generate(printer);
    }
    printer->Print("#endregion\n");
    printer->Print("\n");
  }

  // write children: Messages
  if (file_->message_type_count() > 0) {
    printer->Print("#region Messages\n");
    for (int i = 0; i < file_->message_type_count(); i++) {
      MessageGenerator messageGenerator(file_->message_type(i), this->options());
      messageGenerator.Generate(printer);
    }
    printer->Print("#endregion\n");
    printer->Print("\n");
  }

  // TODO(jtattermusch): add insertion point for services.

  if (!namespace_.empty()) {
    printer->Outdent();
    printer->Print("}\n");
  }
  printer->Print("\n");
  printer->Print("#endregion Designer generated code\n");
}

void ReflectionClassGenerator::WriteIntroduction(io::Printer* printer) {
  printer->Print(
    "// Generated by the protocol buffer compiler.  DO NOT EDIT!\n"
    "// source: $file_name$\n"
    "#pragma warning disable 1591, 0612, 3021\n"
    "#region Designer generated code\n"
    "\n"
    "using pb = global::Google.Protobuf;\n"
    "using pbc = global::Google.Protobuf.Collections;\n"
    "using pbr = global::Google.Protobuf.Reflection;\n"
    "using scg = global::System.Collections.Generic;\n",
    "file_name", file_->name());

  if (!namespace_.empty()) {
    printer->Print("namespace $namespace$ {\n", "namespace", namespace_);
    printer->Indent();
    printer->Print("\n");
  }

  printer->Print(
    "/// <summary>Holder for reflection information generated from $file_name$</summary>\n"
    "$access_level$ static partial class $reflection_class_name$ {\n"
    "\n",
    "file_name", file_->name(),
    "access_level", class_access_level(),
    "reflection_class_name", reflectionClassname_);
  printer->Indent();
}

void ReflectionClassGenerator::WriteDescriptor(io::Printer* printer) {
  printer->Print(
    "#region Descriptor\n"
    "/// <summary>File descriptor for $file_name$</summary>\n"
    "public static pbr::FileDescriptor Descriptor {\n"
    "  get { return descriptor; }\n"
    "}\n"
    "private static pbr::FileDescriptor descriptor;\n"
    "\n"
    "static $reflection_class_name$() {\n",
    "file_name", file_->name(),
    "reflection_class_name", reflectionClassname_);
  printer->Indent();
  printer->Print(
    "byte[] descriptorData = global::System.Convert.FromBase64String(\n");
  printer->Indent();
  printer->Indent();
  printer->Print("string.Concat(\n");
  printer->Indent();

  // TODO(jonskeet): Consider a C#-escaping format here instead of just Base64.
  std::string base64 = FileDescriptorToBase64(file_);
  while (base64.size() > 60) {
    printer->Print("\"$base64$\",\n", "base64", base64.substr(0, 60));
    base64 = base64.substr(60);
  }
  printer->Print("\"$base64$\"));\n", "base64", base64);
  printer->Outdent();
  printer->Outdent();
  printer->Outdent();

  // -----------------------------------------------------------------
  // Invoke InternalBuildGeneratedFileFrom() to build the file.
  printer->Print(
      "descriptor = pbr::FileDescriptor.FromGeneratedCode(descriptorData,\n");
  printer->Print("    new pbr::FileDescriptor[] { ");
  for (int i = 0; i < file_->dependency_count(); i++) {
    // descriptor.proto is special: we don't allow access to the generated code, but there's
    // a separately-exposed property to get at the file descriptor, specifically to allow this
    // kind of dependency.
    if (IsDescriptorProto(file_->dependency(i))) {
      printer->Print("pbr::FileDescriptor.DescriptorProtoFileDescriptor, ");
    } else {
      printer->Print(
      "$full_reflection_class_name$.Descriptor, ",
      "full_reflection_class_name",
      GetReflectionClassName(file_->dependency(i)));
    }
  }
  printer->Print("},\n"
      "    new pbr::GeneratedClrTypeInfo(");
  // Specify all the generated code information, recursively.
  if (file_->enum_type_count() > 0) {
      printer->Print("new[] {");
      for (int i = 0; i < file_->enum_type_count(); i++) {
          printer->Print("typeof($type_name$), ", "type_name", GetClassName(file_->enum_type(i)));
      }
      printer->Print("}, ");
  }
  else {
      printer->Print("null, ");
  }
  if (file_->message_type_count() > 0) {
      printer->Print("new pbr::GeneratedClrTypeInfo[] {\n");
      printer->Indent();
      printer->Indent();
      printer->Indent();
      for (int i = 0; i < file_->message_type_count(); i++) {
          WriteGeneratedCodeInfo(file_->message_type(i), printer, i == file_->message_type_count() - 1);
      }
      printer->Outdent();
      printer->Print("\n}));\n");
      printer->Outdent();
      printer->Outdent();
  }
  else {
      printer->Print("null));\n");
  }

  printer->Outdent();
  printer->Print("}\n");
  printer->Print("#endregion\n\n");
}

// Write out the generated code for a particular message. This consists of the CLR type, property names
// corresponding to fields, names corresponding to oneofs, nested enums, and nested types. Each array part
// can be specified as null if it would be empty, to make the generated code somewhat simpler to read.
// We write a line break at the end of each generated code info, so that in the final file we'll see all
// the types, pre-ordered depth first, one per line. The indentation will be slightly unusual,
// in that it will look like a single array when it's actually constructing a tree, but it'll be easy to
// read even with multiple levels of nesting.
// The "last" parameter indicates whether this message descriptor is the last one being printed in this immediate
// context. It governs whether or not a trailing comma and newline is written after the constructor, effectively
// just controlling the formatting in the generated code.
void ReflectionClassGenerator::WriteGeneratedCodeInfo(const Descriptor* descriptor, io::Printer* printer, bool last) {
  if (IsMapEntryMessage(descriptor)) {
    printer->Print("null, ");
    return;
  }
  // Generated message type
  printer->Print("new pbr::GeneratedClrTypeInfo(typeof($type_name$), $type_name$.Parser, ", "type_name", GetClassName(descriptor));
  
  // Fields
  if (descriptor->field_count() > 0) {
      std::vector<std::string> fields;
      for (int i = 0; i < descriptor->field_count(); i++) {
          fields.push_back(GetPropertyName(descriptor->field(i)));
      }
      printer->Print("new[]{ \"$fields$\" }, ", "fields", JoinStrings(fields, "\", \""));
  }
  else {
      printer->Print("null, ");
  }

  // Oneofs
  if (descriptor->oneof_decl_count() > 0) {
      std::vector<std::string> oneofs;
      for (int i = 0; i < descriptor->oneof_decl_count(); i++) {
          oneofs.push_back(UnderscoresToCamelCase(descriptor->oneof_decl(i)->name(), true));
      }
      printer->Print("new[]{ \"$oneofs$\" }, ", "oneofs", JoinStrings(oneofs, "\", \""));
  }
  else {
      printer->Print("null, ");
  }

  // Nested enums
  if (descriptor->enum_type_count() > 0) {
      std::vector<std::string> enums;
      for (int i = 0; i < descriptor->enum_type_count(); i++) {
          enums.push_back(GetClassName(descriptor->enum_type(i)));
      }
      printer->Print("new[]{ typeof($enums$) }, ", "enums", JoinStrings(enums, "), typeof("));
  }
  else {
      printer->Print("null, ");
  }

  // Nested types
  if (descriptor->nested_type_count() > 0) {
      // Need to specify array type explicitly here, as all elements may be null. 
      printer->Print("new pbr::GeneratedClrTypeInfo[] { ");
      for (int i = 0; i < descriptor->nested_type_count(); i++) {
          WriteGeneratedCodeInfo(descriptor->nested_type(i), printer, i == descriptor->nested_type_count() - 1);
      }
      printer->Print("}");
  }
  else {
      printer->Print("null");
  }
  printer->Print(last ? ")" : "),\n");
}

}  // namespace csharp
}  // namespace compiler
}  // namespace protobuf
}  // namespace google
