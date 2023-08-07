use crate::helpers::{parse_generics, Generics};
use proc_macro::TokenStream;

pub(crate) fn derive(input: TokenStream) -> TokenStream {
    let (
        Generics {
            impl_generics,
            ty_generics,
        },
        mut rest,
    ) = parse_generics(input);
    // This should be the body of the struct `{...}`.
    let last = rest.pop();
    quote! {
        ::kernel::__derive_zeroable!(
            parse_input:
                @sig(#(#rest)*),
                @impl_generics(#(#impl_generics)*),
                @ty_generics(#(#ty_generics)*),
                @body(#last),
        );
    }
}
