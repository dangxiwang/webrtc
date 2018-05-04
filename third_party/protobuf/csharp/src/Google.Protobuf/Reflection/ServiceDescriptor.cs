#region Copyright notice and license
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
#endregion

using System;
using System.Collections.Generic;

namespace Google.Protobuf.Reflection
{
    /// <summary>
    /// Describes a service type.
    /// </summary>
    public sealed class ServiceDescriptor : DescriptorBase
    {
        private readonly ServiceDescriptorProto proto;
        private readonly IList<MethodDescriptor> methods;

        internal ServiceDescriptor(ServiceDescriptorProto proto, FileDescriptor file, int index)
            : base(file, file.ComputeFullName(null, proto.Name), index)
        {
            this.proto = proto;
            methods = DescriptorUtil.ConvertAndMakeReadOnly(proto.Method,
                                                            (method, i) => new MethodDescriptor(method, file, this, i));

            file.DescriptorPool.AddSymbol(this);
        }

        /// <summary>
        /// The brief name of the descriptor's target.
        /// </summary>
        public override string Name { get { return proto.Name; } }

        internal ServiceDescriptorProto Proto { get { return proto; } }

        /// <value>
        /// An unmodifiable list of methods in this service.
        /// </value>
        public IList<MethodDescriptor> Methods
        {
            get { return methods; }
        }

        /// <summary>
        /// Finds a method by name.
        /// </summary>
        /// <param name="name">The unqualified name of the method (e.g. "Foo").</param>
        /// <returns>The method's decsriptor, or null if not found.</returns>
        public MethodDescriptor FindMethodByName(String name)
        {
            return File.DescriptorPool.FindSymbol<MethodDescriptor>(FullName + "." + name);
        }

        /// <summary>
        /// The (possibly empty) set of custom options for this service.
        /// </summary>
        public CustomOptions CustomOptions => Proto.Options?.CustomOptions ?? CustomOptions.Empty;

        internal void CrossLink()
        {
            foreach (MethodDescriptor method in methods)
            {
                method.CrossLink();
            }
        }
    }
}